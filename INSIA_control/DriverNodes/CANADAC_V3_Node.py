"""
Nodo CANADAC V3 con confirmación (ACK) de comandos.

Flujo:
  1. Cada comando se envía con specifier 0x22 y queda registrado como pendiente,
     indexado por (index, sub_index).
  2. El actuador responde con el mismo mensaje y specifier 0x40. Al recibirlo en el topic 'CAN',
     el pendiente se da por confirmado.
  3. Un timer revisa los pendientes. Si vence ack_timeout, reenvía (hasta max_retries veces).
     Qué ocurre al agotar los reintentos depende de la política de la clave (ACK_POLICY):
       - POLICY_REQUIRED (activaciones): alarma. Solo cuenta como confirmación un ACK cuyo valor
         coincide con el último valor enviado; un ACK con otro valor no confirma ni recupera.
       - POLICY_LATEST (consigna de tensión): perder ACK sueltos no importa. Cualquier ACK de esa
         clave, sea del valor que sea, demuestra que el actuador responde. Solo hay alarma si en
         lossy_ack_window segundos no llega ningún ACK de esa clave teniendo comandos pendientes.
     En ningún caso el nodo deja de funcionar: las alarmas se publican en <nombre_nodo>/Alarm.

Concurrencia:
  No se crean hilos ni locks. Todos los callbacks (suscripciones, timers y el servicio de
  parámetros) pertenecen al default_callback_group del nodo, que es MutuallyExclusive. Así nunca
  se ejecutan dos a la vez, ni siquiera con un MultiThreadedExecutor, y el diccionario de
  pendientes solo se toca desde un callback cada vez. Ningún callback bloquea (sin sleep ni
  esperas), de modo que la espera de ACK no frena el envío de nuevos comandos.

Topics (relativos al namespace del nodo):
  Entrada:
    <nombre_nodo>/Target         FloatStamped  consigna de tensión en V (se envía como V*100)
    <nombre_nodo>/EnableRelay    BoolStamped   habilita/deshabilita el relé
    <nombre_nodo>/EnableTension  BoolStamped   activa/desactiva la salida de tensión
    CAN                          CAN           tráfico del bus; aquí llegan los ACK (specifier 0x40)
  Salida:
    <param can>                  CANGroup      comandos hacia el actuador (p. ej. can_control)
    <nombre_nodo>/Alarm          StringStamped 'ALARM ...' al empezar un fallo, 'RECOVERED ...' al acabar
    Heartbeat                    StringStamped latido cada segundo

Parámetros:
  Fijos (se leen al arrancar):
    cobid       cobid del actuador (p. ej. 0x605)
    can         topic por el que se publican los CANGroup
  Modificables en caliente con 'ros2 param set' (ver RUNTIME_PARAMS):
    log_level, ack_timeout, max_retries, check_period, ack_match_data, lossy_ack_window
  Los valores float deben escribirse con decimal en el YAML (0.1, no 1): ROS 2 no permite
  cambiar el tipo de un parámetro en caliente.
"""
import struct
from dataclasses import dataclass
from traceback import format_exc
from typing import Dict, Set, Tuple

import rclpy
from insia_msg.msg import CAN, CANGroup, FloatStamped, BoolStamped
from insia_msg.msg import StringStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header

from INSIA_control.utils.utils import make_can_msg

# Byte 'specifier' del mensaje CAN
SPECIFIER_WRITE = 0x22  # petición de escritura (lo pone make_can_msg; se deja como referencia)
SPECIFIER_ACK = 0x40    # respuesta del actuador: eco de la petición con este specifier

# Identifica cada comando del actuador. Todo el seguimiento (pendientes, alarmas, último valor
# enviado) se hace por clave, de modo que un comando no interfiere con los demás.
Key = Tuple[int, int]  # (index, sub_index)

# Políticas de confirmación por clave (index, sub_index)
POLICY_REQUIRED = 'required'  # ACK obligatorio del valor enviado; alarma al agotar reintentos
POLICY_LATEST = 'latest'      # se toleran pérdidas si llega otro ACK de la clave a tiempo

ACK_POLICY: Dict[Key, str] = {
    (0x0001, 0x04): POLICY_LATEST,    # consigna de tensión
    (0x0002, 0x01): POLICY_REQUIRED,  # activación de tensión
    (0x0003, 0x00): POLICY_REQUIRED,  # enable del relé
}
DEFAULT_POLICY = POLICY_REQUIRED  # cualquier comando nuevo no listado exige ACK
# Al añadir un comando nuevo al nodo, decidir aquí su política:
#   - órdenes puntuales cuyo estado importa (activar, habilitar, cambiar modo) -> POLICY_REQUIRED
#   - valores que se reenvían continuamente y donde solo cuenta el último    -> POLICY_LATEST

# Parámetros modificables en caliente: nombre -> valor por defecto.
# Si no aparecen en el YAML se declaran con estos valores (ver __init__). La validación de cada
# uno está en CANADACNode._parse_runtime_param.
RUNTIME_PARAMS = {
    'ack_timeout': 0.1,      # s. Tiempo de espera del ACK antes de reenviar
    'max_retries': 3,        # reenvíos tras el primer envío (0 = sin reenvíos)
    'check_period': 0.01,    # s. Periodo del timer que revisa los pendientes
    'ack_match_data': True,  # True: el ACK debe traer los mismos 4 bytes de datos
    'lossy_ack_window': 0.5,  # s. Tiempo máximo sin ningún ACK en claves POLICY_LATEST
}


def _data_bytes(data) -> bytes:
    """Normaliza el campo uint8[4] (numpy array, list, bytearray...) a bytes comparables."""
    return bytes(int(b) & 0xFF for b in data)


@dataclass
class PendingCommand:
    """
    Comando enviado que aún no ha recibido su ACK. Hay como máximo uno por clave.

    Se usan tres marcas de tiempo porque cada una responde a una pregunta distinta:
      - value_ns: ¿cuánto tardó en confirmarse este valor? (latencia que aparece en el log)
      - sent_ns:  ¿toca reenviar? (se compara con ack_timeout)
      - since_ns: ¿cuánto lleva el actuador sin responder nada en esta clave? (se compara con
                  la ventana de alarma). No se reinicia al sustituir el comando por otro nuevo.
    Todos los tiempos están en nanosegundos del reloj del nodo (respeta use_sim_time).
    """
    frame: CAN      # último frame enviado para esta clave
    data: bytes     # datos del valor actual
    value_ns: int   # primera transmisión del valor actual (para medir latencia)
    sent_ns: int    # última transmisión (para el timeout)
    since_ns: int   # inicio de la racha sin ninguna respuesta en esta clave (watchdog)
    attempts: int = 1  # transmisiones del valor actual
    exhausted: bool = False  # solo POLICY_LATEST: reintentos agotados, se espera otro ACK


class CANADACNode(Node):
    """
    Driver ROS 2 del actuador ADAC por CAN, con confirmación de comandos.

    Traduce los topics de consigna y habilitación en mensajes CAN, vigila los ACK del actuador,
    reenvía lo que no se confirma y publica alarmas. Ver la cabecera del módulo para el detalle
    del flujo, los topics y los parámetros.
    """

    def __init__(self):
        super().__init__(node_name='CANADAC', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        # --- Logging y parámetros fijos
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.can_connected = self.get_parameter('can').value

        # Parámetros de confirmación. Se declaran si no vienen en el YAML para que
        # 'ros2 param set' funcione igualmente (allow_undeclared_parameters=False).
        for name, default in RUNTIME_PARAMS.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            # Un valor inválido en el YAML aborta el arranque con un mensaje claro
            setattr(self, name, self._parse_runtime_param(name, self.get_parameter(name).value))

        # --- Estado del seguimiento de ACK (solo se toca desde callbacks del nodo)
        self._pending: Dict[Key, PendingCommand] = {}  # comandos esperando ACK
        self._alarmed: Set[Key] = set()  # claves con alarma activa (evita repetir la notificación)
        self._last_sent: Dict[Key, bytes] = {}  # último valor enviado por clave; se conserva
        # aunque el pendiente se borre, para reconocer ACK tardíos del valor correcto

        # Grupo MutuallyExclusive por defecto del nodo; se indica de forma explícita para dejar
        # clara la garantía de exclusión mutua descrita en la cabecera.
        cb_group = self.default_callback_group

        # --- Publishers
        # HistoryPolicy.KEEP_LAST pasado como qos_profile equivale a profundidad 1
        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup, topic=self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        # Profundidad 10 para no perder alarmas que se publiquen seguidas
        self.pub_alarm = self.create_publisher(msg_type=StringStamped, topic=self.get_name() + '/Alarm',
                                               qos_profile=10)

        # --- Suscripciones de comandos
        self.create_subscription(msg_type=FloatStamped, topic=self.get_name() + '/Target', callback=self.consigna,
                                 qos_profile=HistoryPolicy.KEEP_LAST, callback_group=cb_group)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/EnableRelay', callback=self.enable,
                                 qos_profile=HistoryPolicy.KEEP_LAST, callback_group=cb_group)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/EnableTension',
                                 callback=self.enable_tension, qos_profile=HistoryPolicy.KEEP_LAST,
                                 callback_group=cb_group)

        # --- Suscripción a los ACK del actuador.
        # OJO: profundidad 1. Si el topic 'CAN' lleva todo el tráfico del bus pueden perderse ACK
        # y aparecer reintentos o alarmas espurias; en ese caso subir la profundidad (p. ej. 50).
        self.create_subscription(msg_type=CAN, topic='CAN', callback=self.msg_can,
                                 qos_profile=HistoryPolicy.KEEP_LAST, callback_group=cb_group)

        # --- Timers: latido y revisión periódica de pendientes (reintentos y alarmas)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat, callback_group=cb_group)
        self.timer_ack = self.create_timer(self.check_period, self.check_pending, callback_group=cb_group)

        # Se registra al final para que los declare_parameter anteriores no lo disparen
        # antes de que existan los timers.
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.logger.info(f'ACK: cobid=0x{self.cobid:03X} timeout={self.ack_timeout}s '
                         f'max_retries={self.max_retries} check_period={self.check_period}s '
                         f'match_data={self.ack_match_data} lossy_window={self.lossy_ack_window}s')

    # ------------------------------------------------------------------ parámetros

    @staticmethod
    def _parse_runtime_param(name, value):
        """
        Valida y convierte un parámetro de RUNTIME_PARAMS. Lanza ValueError si no es válido.

        Se usa tanto al arrancar como en parameters_callback, para que las reglas sean las mismas.
        Los tiempos deben ser > 0 y se convierten a float (admite enteros); max_retries admite 0
        (sin reenvíos). Se rechaza bool en los numéricos porque en Python bool es subclase de int.
        """
        if name in ('ack_timeout', 'check_period', 'lossy_ack_window'):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} debe ser numérico (recibido {value!r})')
            value = float(value)
            if value <= 0.0:
                raise ValueError(f'{name} debe ser > 0 (recibido {value})')
            return value
        if name == 'max_retries':
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'max_retries debe ser entero (recibido {value!r})')
            if value < 0:
                raise ValueError(f'max_retries debe ser >= 0 (recibido {value})')
            return value
        if name == 'ack_match_data':
            if not isinstance(value, bool):
                raise ValueError(f'ack_match_data debe ser booleano (recibido {value!r})')
            return value
        raise ValueError(f'Parámetro desconocido {name}')

    def parameters_callback(self, params):
        """
        Callback de 'ros2 param set': valida y aplica los cambios de parámetros en caliente.

        Es todo o nada: si algún parámetro del lote no es válido se rechaza el lote completo
        (successful=False) y no se aplica ninguno. Si cambia check_period se recrea el timer de
        revisión. Los cambios de ack_timeout, max_retries, etc. afectan también a los comandos
        que ya estaban pendientes, a partir del siguiente ciclo del timer.
        Los parámetros fijos (cobid, can) no se tratan aquí: cambiarlos requiere reiniciar.
        """
        # Primero se valida todo; solo se aplica si todos los valores son correctos
        updates = {}
        new_log_level = None
        try:
            for param in params:
                if param.name == 'log_level':
                    new_log_level = param.value
                elif param.name in RUNTIME_PARAMS:
                    updates[param.name] = self._parse_runtime_param(param.name, param.value)
        except ValueError as e:
            self.logger.warning(f'Parámetro rechazado: {e}')
            return SetParametersResult(successful=False, reason=str(e))

        if new_log_level is not None:
            self.logger.set_level(new_log_level)

        for name, value in updates.items():
            setattr(self, name, value)
            self.logger.info(f'Parámetro {name} = {value}')

        if 'check_period' in updates:
            # Ejecutándose en el mismo grupo exclusivo, el timer no puede estar en marcha ahora
            self.destroy_timer(self.timer_ack)
            self.timer_ack = self.create_timer(self.check_period, self.check_pending,
                                               callback_group=self.default_callback_group)

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ comandos

    def enable_tension(self, data):
        """
        Callback de <nodo>/EnableTension: activa (True) o desactiva (False) la salida de tensión.

        Objeto CAN: index 0x0002, sub_index 0x01, dato 0x01/0x00.
        Política POLICY_REQUIRED: alarma si el actuador no confirma exactamente el valor enviado.
        """
        if data.data:
            self.logger.debug('Enviar activacion de tension')
            value = 0x01
        else:
            self.logger.debug('Enviar desactivacion de tension')
            value = 0x00
        self._send(make_can_msg(node=self.cobid, index=0x0002, sub_index=0x01, data=value,
                                clock=self.get_clock().now().to_msg()))

    def enable(self, data):
        """
        Callback de <nodo>/EnableRelay: habilita (True) o deshabilita (False) el relé.

        Objeto CAN: index 0x0003, sub_index 0x00, dato 0x01/0x00.
        Política POLICY_REQUIRED: alarma si el actuador no confirma exactamente el valor enviado.
        """
        if data.data:
            self.logger.debug('Enviar enable')
            value = 0x01
        else:
            self.logger.debug('Desactivar enable')
            value = 0x00
        # sub_index 0x00 explícito: es el valor que se usaba por defecto. Verificar en el
        # manual del actuador que el objeto 0x0003 se escribe en el sub_index 0.
        self._send(make_can_msg(node=self.cobid, index=0x0003, sub_index=0x00, data=value,
                                clock=self.get_clock().now().to_msg()))

    def consigna(self, data):
        """
        Callback de <nodo>/Target: envía la consigna de tensión.

        Objeto CAN: index 0x0001, sub_index 0x04. El valor en voltios se envía como entero
        en centésimas (2.5 V -> 250), codificado como int32 little-endian.
        Política POLICY_LATEST: se toleran pérdidas sueltas de ACK (ver cabecera del módulo).
        """
        self.logger.debug(f'Tension recibida {data.data}')
        try:
            frame = make_can_msg(node=self.cobid, index=0x0001, sub_index=0x04, data=int(data.data * 100),
                                 clock=self.get_clock().now().to_msg())
        except (struct.error, ValueError, OverflowError) as e:
            # Evita que una consigna fuera de rango tumbe el nodo
            self.logger.error(f'Consigna no codificable {data.data}: {e}')
            return
        self._send(frame)

    # ------------------------------------------------------------------ envío y ACK

    def _now_ns(self) -> int:
        """Tiempo actual del reloj del nodo en ns (usa el tiempo simulado si use_sim_time)."""
        return self.get_clock().now().nanoseconds

    def _publish_frames(self, frames):
        """
        Publica varios frames en un único CANGroup, con marca de tiempo nueva.

        Actualiza también la marca de tiempo de cada frame, que en un reintento pasa a ser la
        del reenvío y no la del envío original.
        """
        stamp = self.get_clock().now().to_msg()
        for frame in frames:
            frame.header.stamp = stamp
        self.pub_CAN.publish(CANGroup(header=Header(stamp=stamp), can_frames=list(frames)))

    def _send(self, frame: CAN, track=True):
        """
        Punto único de envío de comandos al actuador.

        Publica el frame y, si track=True, lo registra como pendiente de ACK. Si ya había un
        pendiente para la misma clave, el nuevo lo sustituye: se reinician los intentos y el
        estado 'exhausted', pero se conserva since_ns (ver comentario interno).
        Usar track=False solo para mensajes de los que el actuador no devuelve eco.

        :param frame: mensaje CAN creado con make_can_msg
        :param track: True para esperar confirmación del actuador
        """
        if track:
            now_ns = self._now_ns()
            key = (frame.index, frame.sub_index)
            data = _data_bytes(frame.data)
            self._last_sent[key] = data
            entry = self._pending.get(key)
            if entry is None:
                self._pending[key] = PendingCommand(frame=frame, data=data, value_ns=now_ns,
                                                    sent_ns=now_ns, since_ns=now_ns)
            else:
                # Solo importa el último valor: el nuevo sustituye al pendiente. Se mantiene
                # since_ns para que el watchdog detecte un actuador mudo aunque lleguen
                # comandos más rápido que ack_timeout.
                if entry.data != data:
                    entry.value_ns = now_ns
                entry.frame = frame
                entry.data = data
                entry.sent_ns = now_ns
                entry.attempts = 1
                entry.exhausted = False
        self._publish_frames([frame])
        self.logger.debug(f'TX index=0x{frame.index:04X} sub=0x{frame.sub_index:02X} '
                          f'data={_data_bytes(frame.data).hex()}')

    def msg_can(self, msg: CAN):
        """
        Callback del topic 'CAN': procesa los ACK del actuador.

        Descarta todo lo que no sea un ACK (specifier 0x40) de nuestro cobid. Para un ACK de una
        clave (index, sub_index) hay tres casos:
          1. El valor coincide con el último enviado (o ack_match_data=False): confirmación.
             Se borra el pendiente, si lo hay, y se recupera la alarma de la clave.
          2. Valor distinto en una clave POLICY_LATEST: el actuador está vivo. Se reinicia la
             ventana de alarma y se recupera la alarma, pero el valor actual sigue pendiente.
          3. Valor distinto en una clave POLICY_REQUIRED: no cuenta para nada. El actuador no ha
             confirmado lo que se le pidió.
        """
        try:
            # Filtro rápido: el bus trae mucho tráfico ajeno a este actuador
            if msg.is_extended or msg.cobid != self.cobid or msg.specifier != SPECIFIER_ACK:
                return

            key = (msg.index, msg.sub_index)
            data = _data_bytes(msg.data)
            entry = self._pending.get(key)
            policy = ACK_POLICY.get(key, DEFAULT_POLICY)
            last = self._last_sent.get(key)
            matches = (not self.ack_match_data) or data == last
            tag = f'index=0x{key[0]:04X} sub=0x{key[1]:02X} data={data.hex()}'

            if matches:
                if entry is not None:
                    del self._pending[key]
                    latency_ms = (self._now_ns() - entry.value_ns) / 1e6
                    self.logger.debug(f'ACK {tag} latencia={latency_ms:.1f} ms intentos={entry.attempts}')
                else:
                    # Duplicado, o ACK tardío tras agotar reintentos: el valor sí llegó
                    self.logger.debug(f'ACK sin pendiente {tag}')
                self._clear_alarm(key)
            elif policy == POLICY_LATEST:
                # ACK de otro valor de tensión (p. ej. uno ya sustituido): el actuador responde,
                # así que se reinicia la ventana y se da por recuperado. El valor actual sigue
                # pendiente y se reintentará si hace falta.
                if entry is not None:
                    entry.since_ns = self._now_ns()
                self.logger.debug(f'ACK de otro valor {tag} (último enviado '
                                  f'{last.hex() if last else "-"}): se tolera')
                self._clear_alarm(key)
            else:
                # POLICY_REQUIRED: el actuador confirma un valor distinto al último pedido.
                # No confirma, no reinicia la ventana y no recupera la alarma.
                self.logger.debug(f'ACK {tag} no coincide con el último valor enviado '
                                  f'{last.hex() if last else "-"}')
        except ValueError as e:
            self.logger.debug(f'{e}')
        except Exception as e:
            self.logger.error(f'Error procesando ACK: {e}\n{format_exc()}')

    def check_pending(self):
        """
        Timer (cada check_period): reenvía los pendientes vencidos y decide las alarmas.

        Para cada pendiente se comprueban dos cosas:
          a) Timeout por mensaje: si pasa ack_timeout sin ACK se reenvía, hasta max_retries veces.
             Al agotarlos, POLICY_REQUIRED -> alarma y se borra el pendiente;
             POLICY_LATEST -> se deja de reenviar (exhausted) y se espera a la ventana.
          b) Ventana sin respuesta: tiempo desde since_ns sin ningún ACK válido en la clave.
             POLICY_REQUIRED usa ack_timeout*(max_retries+1); POLICY_LATEST usa lossy_ack_window.

        Ejemplo con los valores por defecto (timeout 0.1 s, 3 reintentos, ventana 0.5 s) y un
        actuador que no responde:
          t=0.0 envío | t=0.1, 0.2, 0.3 reintentos 1, 2 y 3 | t=0.4 reintentos agotados
            - activación: ALARM en t=0.4
            - tensión:    sin alarma todavía; ALARM en t=0.5 si no ha llegado ningún ACK de
                          tensión (de ningún valor). Si llega uno antes, no pasa nada.
        Los instantes reales se retrasan hasta un check_period por la resolución del timer.
        """
        if self.shutdown_flag or not self._pending:
            return
        now_ns = self._now_ns()
        timeout_ns = int(self.ack_timeout * 1e9)
        resend = []

        for key, entry in list(self._pending.items()):
            lossy = ACK_POLICY.get(key, DEFAULT_POLICY) == POLICY_LATEST

            # a) Timeout por mensaje (no aplica si ya se agotaron los reintentos)
            if not entry.exhausted and now_ns - entry.sent_ns >= timeout_ns:
                if entry.attempts <= self.max_retries:
                    entry.attempts += 1
                    entry.sent_ns = now_ns
                    resend.append(entry.frame)
                    self.logger.debug(f'Reintento {entry.attempts - 1}/{self.max_retries} '
                                      f'index=0x{key[0]:04X} sub=0x{key[1]:02X} data={entry.data.hex()}')
                elif lossy:
                    # Pérdida tolerada: se deja de reenviar, pero la entrada se conserva para que
                    # la ventana decida si hay alarma cuando no llegue ningún otro ACK
                    entry.exhausted = True
                    self.logger.debug(f'Sin ACK tras {entry.attempts} intentos index=0x{key[0]:04X} '
                                      f'sub=0x{key[1]:02X} data={entry.data.hex()}: tolerado')
                else:
                    del self._pending[key]
                    self._raise_alarm(key, entry, 'reintentos agotados')
                    continue

            # b) Ventana sin ninguna respuesta en esta clave. En POLICY_REQUIRED cubre comandos que
            # se sustituyen antes de su timeout; en POLICY_LATEST es el único motivo de alarma.
            window_ns = int(self.lossy_ack_window * 1e9) if lossy else timeout_ns * (self.max_retries + 1)
            if now_ns - entry.since_ns >= window_ns:
                self._raise_alarm(key, entry, f'sin ningún ACK en {window_ns / 1e9:.3f} s')
                if entry.exhausted:
                    del self._pending[key]
                    continue
                # El pendiente sigue vivo (llegan comandos nuevos): se rearma la ventana para no
                # evaluar la alarma en cada ciclo. _raise_alarm no repite la notificación.
                entry.since_ns = now_ns

        if resend:
            # Un único CANGroup por ciclo: con profundidad 1 en el publisher, varias
            # publicaciones seguidas podrían pisarse
            self._publish_frames(resend)

    # ------------------------------------------------------------------ alarmas

    def _publish_alarm(self, text):
        """Publica un texto en el topic <nodo>/Alarm con la marca de tiempo actual."""
        msg = StringStamped(data=text)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_alarm.publish(msg)

    def _raise_alarm(self, key: Key, entry: PendingCommand, reason: str):
        """
        Notifica el fallo de una clave. Solo publica en la transición a fallo: si la clave ya
        tenía alarma activa, se limita a un log debug. La alarma se mantiene hasta que
        _clear_alarm la recupera al llegar un ACK válido.

        Formato: 'ALARM <nodo> cobid=0x... index=0x... sub_index=0x... data=<hex> intentos=N: <motivo>'
        """
        text = (f'ALARM {self.get_name()} cobid=0x{self.cobid:03X} index=0x{key[0]:04X} '
                f'sub_index=0x{key[1]:02X} data={entry.data.hex()} intentos={entry.attempts}: {reason}')
        if key in self._alarmed:
            # Ya notificada: no se inunda el topic mientras dure el fallo
            self.logger.debug(text)
            return
        self._alarmed.add(key)
        self.logger.warning(text)
        self._publish_alarm(text)

    def _clear_alarm(self, key: Key):
        """
        Cierra la alarma de una clave, si la tenía, publicando 'RECOVERED ...'.
        Se llama con cada ACK válido; si la clave no estaba en alarma no hace nada.
        """
        if key not in self._alarmed:
            return
        self._alarmed.discard(key)
        text = (f'RECOVERED {self.get_name()} cobid=0x{self.cobid:03X} index=0x{key[0]:04X} '
                f'sub_index=0x{key[1]:02X}')
        self.logger.info(text)
        self._publish_alarm(text)

    # ------------------------------------------------------------------ varios

    def publish_heartbeat(self):
        """Timer (1 s): publica el nombre del nodo en Heartbeat para supervisar que sigue vivo."""
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(msg)

    def shutdown(self):
        """Detiene los timers y descarta los pendientes. Se llama desde main al terminar."""
        try:
            self.shutdown_flag = True
            self.timer_heartbeat.cancel()
            self.timer_ack.cancel()
            self._pending.clear()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    """Punto de entrada: crea el nodo, lo ejecuta y garantiza un cierre limpio."""
    rclpy.init(args=args)
    manager = None
    try:
        manager = CANADACNode()
        rclpy.spin(manager)
    except (KeyboardInterrupt, ExternalShutdownException):
        if manager is not None:
            print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
        print(format_exc())
    finally:
        if manager is not None:
            manager.shutdown()
            manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
