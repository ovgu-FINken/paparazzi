"""
Using an IvyServer
------------------

The following code is a typical example of use:

.. python::
    from ivy.ivy import IvyServer
    
    class MyAgent(IvyServer):
      def __init__(self, name):
        IvyServer.__init__(self,'MyAgent')
        self.name = name
        self.start('127.255.255.255:2010')
        self.bind_msg(self.handle_hello, 'hello .*')
        self.bind_msg(self.handle_button, 'BTN ([a-fA-F0-9]+)')
        
      def handle_hello(self, agent):
        print '[Agent %s] GOT hello from %r'%(self.name, agent)
      
      def handle_button(self, agent, btn_id):
        print '[Agent %s] GOT BTN button_id=%s from %r'%(self.name, btn_id, agent)
        # let's answer!
        self.send_msg('BTN_ACK %s'%btn_id)
    
    a=MyAgent('007')


Implementation details
----------------------

An Ivy client is made of several threads:

  - an `IvyServer` instance

  - a UDP server, launched by the Ivy server, listening to incoming UDP
    broadcast messages

  - `IvyTimer` objects 

:group Messages types: BYE, ADD_REGEXP, MSG, ERROR, DEL_REGEXP, END_REGEXP,
  END_INIT, START_REGEXP, START_INIT, DIRECT_MSG, DIE
:group Separators: ARG_START, ARG_END
:group Misc. constants: DEFAULT_IVYBUS, PROTOCOL_VERSION, IVY_SHOULD_NOT_DIE
  IvyApplicationConnected, IvyApplicationDisconnected, DEFAULT_TTL
:group Objects and functions related to logging: ivylogger, debug, log, warn,
  error, ivy_loghdlr, ivy_logformatter

Copyright (c) 2005-2016 Sebastien Bigaret <sbigaret@users.sourceforge.net>
"""
import sys
# The next line taken from https://pythonhosted.org/six/
# Copyright (c) 2010-2016 Benjamin Peterson
PY2 = sys.version_info[0] == 2

import logging
import os
import random
import re
import socket
import struct
import threading
import time
import traceback
import types
if PY2:
    import SocketServer as socketserver
else:
    import socketserver

ivylogger = logging.getLogger('Ivy')

if os.environ.get('IVY_LOG_TRACE'):
    logging.TRACE = logging.DEBUG - 1
    logging.addLevelName(logging.TRACE, 'TRACE')
    trace = lambda *args, **kw: ivylogger.log(logging.TRACE, *args, **kw)
else:
    trace = lambda *args, **kw: None

debug = ivylogger.debug
info = log = ivylogger.info
warn = ivylogger.warning
error = ivylogger.error

ivy_loghdlr = logging.StreamHandler()  # stderr by default
ivy_logformatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')

ivy_loghdlr.setFormatter(ivy_logformatter)
ivylogger.addHandler(ivy_loghdlr)
ivylogger.setLevel(logging.INFO)

##
DEFAULT_IVYBUS = '127:2010'
PROTOCOL_VERSION = 3

# Message types. Refer to "The Ivy architecture and protocol" for details
BYE = 0
ADD_REGEXP = 1
MSG = 2
ERROR = 3
DEL_REGEXP = 4

# START_REGEXP and END_REGEXP are the ones declared in ivy.c
# however we'll use the aliases START_INIT and END_INIT here
END_REGEXP = END_INIT = 5
START_REGEXP = START_INIT = 6

DIRECT_MSG = 7
DIE = 8

PING = 9
PONG = 10

# Other constants
ARG_START = '\002'
ARG_END = '\003'

# for multicast, arbitrary TTL value taken from ivysocket.c:SocketAddMember
DEFAULT_TTL = 64

IvyApplicationConnected = 1
IvyApplicationDisconnected = 2

IvyRegexpAdded = 3
IvyRegexpRemoved = 4

IVY_SHOULD_NOT_DIE = 'Ivy Application Should Not Die'


def void_function(*arg, **kw):
    """A function that accepts any number of parameters and does nothing"""
    pass

def UDP_init_and_listen(broadcast_addr, port, socket_server):
    """
    Called by an IvyServer at startup; the method is responsible for:

    - sending the initial UDP broadcast message,

    - waiting for incoming UDP broadcast messages being sent by new clients
      connecting on the bus.  When it receives such a message, a connection
      is established to that client and that connection (a socket) is then
      passed to the IvyServer instance.

    :Parameters:
      - `broadcast_addr`: the broadcast address used on the Ivy bus
      - `port`: the port dedicated to the Ivy bus
      - `socket_server`: instance of an IvyServer handling communications
        for our client.
    """
    log('Starting Ivy UDP Server on %r:%r' % (broadcast_addr, port))

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    on = 1
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, on)
    if hasattr(socket, 'SO_REUSEPORT'):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, on)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, on)

    s.bind(('', port))  # '' means: INADDR_ANY

    # Multicast
    if is_multicast(broadcast_addr):
        debug('Broadcast address is a multicast address')

        ifaddr = socket.INADDR_ANY
        mreq = struct.pack('4sl',
                           socket.inet_aton(broadcast_addr),
                           socket.htonl(ifaddr))
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, DEFAULT_TTL)
    # /Multicast

    msg="%li %s %s %s\n"%(PROTOCOL_VERSION,
                          socket_server.port, socket_server.agent_id,
                          socket_server.agent_name)
    if PY2:
        s.sendto(msg, (broadcast_addr, port))
    else:
        s.sendto(msg.encode(), (broadcast_addr, port))

    s.settimeout(0.1)
    while socket_server.isAlive():
        try:
            udp_msg, (ip, ivybus_port) = s.recvfrom(1024)
        except socket.timeout:
            continue

        if not PY2:
            udp_msg=udp_msg.decode('UTF-8')
        debug('UDP got: %r from: %r', udp_msg, ip)

        appid = appname = None
        try:
            udp_msg_l = udp_msg.split(' ')
            protocol_version, port_number = udp_msg_l[:2]
            if len(udp_msg_l) > 2:
                # "new" udp protocol, with id & appname
                appid = udp_msg_l[2]
                appname = ' '.join(udp_msg_l[3:]).strip('\n')
                debug('IP %s has id: %s and name: %s', ip, appid, appname)
            else:
                debug('Received message w/o app. id & name from %r', ip)

            port_number = int(port_number)
            protocol_version = int(protocol_version)

        except ValueError:  # unpack error, invalid literal for int()
            warn('Received an invalid UDP message (%r) from :', udp_msg)

        if protocol_version != PROTOCOL_VERSION:
            error('Received a UDP broadcast msg. w/ protocol version:%s , expected: %s', protocol_version,
                  PROTOCOL_VERSION)
            continue

        if appid == socket_server.agent_id:
            # this is us!
            debug('UDP from %r: ignored: we sent that one!', ip)
            continue

        # build a new socket and delegate its handling to the SocketServer
        debug('New client connected: %s:%s', ip, port_number)

        new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        new_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, on)
        trace('New client %s:%s, socket %r', ip, port_number, new_socket)
        # Since we already have a client's name and id, lets register it
        # (this was previously done in IvyHandler's __init__() only)
        # but we want to check that we did not receive more than once a
        # broadcast coming from the same 
        new_client = socket_server._get_client(ip, port_number, new_socket,
                                               appid, appname)
        if new_client is None:
            # an agent with that app-id is already registered
            info('UDP from %s:%s (%s): discarding message, an application w/ id=%s is already registered', ip,
                 port_number, appname, appid)
            continue

        try:
            new_socket.connect((ip, port_number))
        except:
            # e.g., timeout on connect
            info('Client %r: failed to connect to its socket, ignoring it',
                 new_client)
            debug('Client %r: failed to connect to its socket, got:%s',
                  new_client, traceback.format_exc())
            socket_server.remove_client(ip, port_number,
                                        trigger_application_callback=False)
        else:
            socket_server.process_request(new_socket, (ip, port_number))
    log('UDP Server stopped')


def is_multicast(ip):
    """
    Tells whether the specified ip is a multicast address or not

    :param ip: an IPv4 address in dotted-quad string format, for example
      192.168.2.3
    """
    return int(ip.split('.')[0]) in range(224, 239)


def decode_ivybus(ivybus=None):
    """
    Transforms the supplied string into the corresponding broadcast address
    and port

    :param ivybus: if ``None`` or empty, defaults to environment variable
      ``IVYBUS``

    :return: a tuple made of (broadcast address, port number). For example:
      ::

        >>> print decode_ivybus('192.168.12:2010')
        ('192.168.12.255', 2010)

    """
    if not ivybus:
        ivybus = os.getenv('IVYBUS', DEFAULT_IVYBUS)

    broadcast, port = ivybus.split(':', 1)
    port = int(port)
    broadcast = broadcast.strip('.')
    broadcast += '.' + '.'.join(['255', ] * (4 - len(broadcast.split('.'))))
    # if broadcast is multicast it had 4 elements -> previous line added a '.'
    broadcast = broadcast.strip('.')
    debug('Decoded ivybus %s:%s', broadcast, port)
    return broadcast, port


def decode_msg(msg):
    """

    :return: msg_id, numerical_id, parameters
    :except IvyMalformedMessage:
    """
    try:
        msg_id, _msg = msg.split(' ', 1)
        msg_id = int(msg_id)
        num_id, params = _msg.split(ARG_START, 1)
        num_id = int(num_id)

        if ARG_END in params:
            # there is an extra ARG_END after the last parameter and
            # before the newline
            params = params[:-1].split(ARG_END)

    except ValueError:
        raise IvyMalformedMessage
    return msg_id, num_id, params


def encode_message(msg_type, numerical_id, params=''):
    """

    params is string -> added as-is
    params is list -> concatenated, separated by ARG_END
    """
    msg = "%s %s" % (msg_type, numerical_id) + ARG_START

    _type_str=None
    if PY2:
        _type_str=basestring
    else:
        _type_str=str

    if isinstance(params, _type_str):
        msg += params
    else:
        msg += ARG_END.join(params)
        msg += ARG_END
    trace('encode_message(params: %s) -> %s' % (repr(params), repr(msg + '\n')))

    if PY2:
        msg = msg + '\n'
    else:
        msg = ( msg + '\n' ).encode()

    return msg


class IvyProtocolError(Exception):
    pass


class IvyMalformedMessage(Exception):
    pass


class IvyIllegalStateError(RuntimeError):
    pass

NOT_INITIALIZED = 0
INITIALIZATION_IN_PROGRESS=1
INITIALIZED=2

class IvyClient:
    """
    Represents a client connected to the bus. Every callback methods
    registered by an agent receive an object of this type as their first
    parameter, so that they know which agent on the bus is the cause of the
    event which triggered the callback.    

    An IvyClient is responsible for:

      - managing the remote agent's subscriptions,

      - sending messages to the remote agent.

    It is **not** responsible for receiving messages from the client: another
    object is in charge of that, namely an `IvyHandler` object.

    The local IvyServer creates one IvyClient per agent on the bus.

    MT-safety
    ---------
    See the discussion in `regexps`.

    :group Protocol-related methods: start_init, end_init,
      send_new_subscription, remove_subscription, wave_bye
    :group Manipulating the remote agent's subscriptions:
      add_regexp, remove_regexp
    :group Sending messages: send_msg, send_direct_message, send_die_message

    :ivar regexps: a dictionary mapping subscriptions' ids (as delivered by
      `add_regexp`) to the corresponding regular expressions. Precisely, it
      maps ids to tuples being ``(regexp_as_string, compiled_regexp)``.  You
      shouldn't directly access or manipulate this variable, use `add_regexp`
      and `remove_regexp` instead; however if you really need/want to, you
      must acquire the `regexp_lock` before accessing/modifying it.

    :ivar regexp_lock: a non-reentrant lock protecting the variable
      `regexps`. Used by methods `add_regexp`, `remove_regexp` and `send_msg`.
    :type regexp_lock: `threading.Lock`
    """

    def __init__(self, ip, port, client_socket,
                 agent_id=None, agent_name=None):
        self.agent_id = agent_id
        # agent_name will be overridden when start_init() is called
        # but nevermind,
        self.agent_name = agent_name
        self.ip = ip
        self.port = port
        self.socket = client_socket

        self.regexps = {}  # maps regexp_id to (regexp_string, compiled_regexp)

        self.fqdn = socket.getfqdn(ip)
        self.status = NOT_INITIALIZED
        self.socket.settimeout(0.1)

        # regexps are modified w/ add_regexp and remove_regexp, called
        # by the server, while they are accessed by send_message()
        # called by the corresponding IvyHandler-thread --> needs to be
        # protected against concurrent access.
        self.regexps_lock = threading.Lock()  # non-reentrant, faster than RLock

        self.ping_ts = [] # timestamps
        self.ping_lock = threading.Lock()

    def start_init(self, agent_name):
        """
        Finalizes the initialization process by setting the client's
        agent_name.  This is a Ivy protocol requirement that an application
        sends its agent-name only once during the initial handshake (beginning
        with message of type ``START_INIT`` and ending with a type
        ``END_INIT``).  After this method is called, we expect to receive the
        initial subscriptions for that client (or none); the initialization
        process completes after `end_init` is called.
        
        :except IvyIllegalStateError: if the method has already been called
          once
        """
        if self.status != NOT_INITIALIZED:
            raise IvyIllegalStateError
        self.agent_name = agent_name
        self.status = INITIALIZATION_IN_PROGRESS
        debug('Client:%r: Starting initialization', self)

    def end_init(self):
        """
        Should be called when the initialization process ends.
        
        :except IvyIllegalStateError: if the method has already been called
          (and ``self.status`` has already been set to ``INITIALIZED``)
        """
        if self.status == INITIALIZED:
            raise IvyIllegalStateError
        debug('Client:%r: Initialization ended', self)
        self.status = INITIALIZED

    def add_regexp(self, regexp_id, regexp):
        """
        :except IvyIllegalStateError: if the client has not been fully
          initialized yet (see `start_init`)
        """
        if self.status not in (INITIALIZATION_IN_PROGRESS, INITIALIZED):
            # initialization has not begun
            raise IvyIllegalStateError

        # TODO: handle error on compile
        debug('Client:%r: Adding regexp id=%s: %r', self, regexp_id, regexp)
        self.regexps_lock.acquire()
        try:
            self.regexps[regexp_id] = (regexp, re.compile(regexp))
        finally:
            self.regexps_lock.release()

    def remove_regexp(self, regexp_id):
        """
        Removes a regexp
        
        :return: the regexp that has been removed
        :except IvyIllegalStateError: if the client has not been fully
          initialized yet (see `start_init`)
        :except KeyError: if no such subscription exists
        """
        if self.status not in (INITIALIZATION_IN_PROGRESS, INITIALIZED):
            # initialization has not begun
            raise IvyIllegalStateError
        debug('Client:%r: removing regexp id=%s', self, regexp_id)
        regexp = None

        self.regexps_lock.acquire()
        try:
            regexp = self.regexps.pop(regexp_id)[0]
        finally:
            self.regexps_lock.release()
        return regexp

    def get_regexps(self):
        self.regexps_lock.acquire()
        try:
            return [(idx, s[0]) for idx, s in self.regexps.items()]
        finally:
            self.regexps_lock.release()

    def send_msg(self, msg):
        """
        Sends the message to the client.  The client will receive one message
        for each of its subscriptions (regexps) that the message matches.
        
        :return: ``True`` if the message was actually sent to the client, that
          is: if there is one or more regexps matching the message in the
          client's subscriptions; returns ``False`` otherwise.
        
        """
        if self.status != INITIALIZED:
            return
        debug('Client:%r: Searching subscriptions matching msg %r', self, msg)
        # For each regexp that the msg matches, send the corresponding message
        result = False
        self.regexps_lock.acquire()
        try:

            for num_id, (s, r) in self.regexps.items():
                captures = r.match(msg)
                if captures:
                    # Value for an optional group not participating in the
                    # match defaults to the empty string
                    captures = captures.groups(default='')

                    # The following is needed to reproduce the very same
                    # behaviour observed w/ the C library
                    # (tested w/ pyhello.py and ivyprobe)
                    if len(captures) == 0:
                        captures = ''
                    debug('Client:%r: msg being sent: %r (regexp: %r)',
                          self, captures, s)
                    self._send(MSG, num_id, captures)
                    result = True
            return result
        finally:
            self.regexps_lock.release()

    def send_direct_message(self, num_id, msg):
        """
        Sends a direct message

        Note: the message will be encoded by `encode_message` with
        ``numerical_id=num_id`` and ``params==msg``; this means that if `msg`
        is not a string but a list or a tuple, the direct message will contain
        more than one parameter.  This is an **extension** of the original Ivy
        design, supported by python, but if you want to inter-operate with
        applications using the standard Ivy API the message you send *must* be
        a string. See in particular in ``ivy.h``::
        
          typedef void (*MsgDirectCallback)( IvyClientPtr app, void *user_data, int id, char *msg ) ;

        """
        if self.status == INITIALIZED:
            debug('Client:%r: a direct message being sent: id: %r msg: %r',
                  self, num_id, msg)
            self._send(DIRECT_MSG, num_id, msg)

    def send_die_message(self, num_id=0, msg=''):
        """
        Sends a die message
        """
        if self.status == INITIALIZED:
            debug('Client:%r: die msg being sent: num_id: %r msg: %r',
                  self, num_id, msg)
            self._send(DIE, num_id, msg)

    def send_new_subscription(self, idx, regexp):
        """
        Notifies the remote agent that we (the local agent) subscribe to
        a new type of messages

        :Parameters:
          - `idx`: the index/id of the new subscription. It is the
            responsibility of the local agent to make sure that every
            subscription gets a unique id.
          - `regexp`: a regular expression. The subscription consists in
            receiving messages matching the regexp.
        """
        self._send(ADD_REGEXP, idx, regexp)

    def send_ping(self):
        """
        """
        self.ping_lock.acquire()
        try:
            self.ping_ts.append(time.time())
            self._send(PING, 0)
        finally:
            self.ping_lock.release()

    def get_next_ping_delta(self):
        """
        """
        self.ping_lock.acquire()
        try:
            if len(self.ping_ts)==0:
                return None
            ts0 = self.ping_ts.pop(0)
            return time.time() - ts0
        finally:
            self.ping_lock.release()

    def remove_subscription(self, idx):
        """
        Notifies the remote agent that we (the local agent) are not
        interested in a given subscription.

        :Parameters:
          - `idx`: the index/id of a subscription previously registered with
            `send_new_subscription`.
        """
        self._send(DEL_REGEXP, idx)

    def wave_bye(self, num_id=0):
        """Notifies the remote agent that we are about to quit"""
        self._send(BYE, num_id)

    def send_error(self, num_id, msg):
        """
        Sends an error message
        """
        self._send(ERROR, num_id, msg)

    def __eq__(self, client):
        """
        cf. dict[client] or dict[(ip,port)] UNNEEDED FOR THE MOMENT
        """
        if isinstance(client, IvyClient):
            return self.ip == client.ip and self.port == client.port

        _tuple_list=None
        if PY2:
            _tuple_list=(types.TupleType, types.ListType)
        else:
            _tuple_list=(tuple, list)

        if type(client) in _tuple_list and len(client) == 2:
            return self.ip == client[0] and self.port == client[1]

        return False

    def __hash__(self):
        """``hash((self.ip, self.port))``"""
        return hash((self.ip, self.port))

    def __repr__(self):
        """Returns ``'ip:port (agent_name)'``"""
        return '%s:%s (%s)' % (self.ip, self.port, self.agent_name)

    def __str__(self):
        """Returns ``'agent_name@FQDN'``"""
        return '%s@%s' % (self.agent_name, self.fqdn)

    def _send(self, msg_type, *params):
        """
        Internally used to send message to the remote agent through the opened
        socket `self.socket`.  This method catches all exceptions
        `socket.error` and `socket.timeout` and ignores them, simply logging
        them at the "info" level.

        The errors that can occur are for example::

            socket.timeout: timed out
            socket.error: (104, 'Connection reset by peer')
            socket.error: (32, 'Broken pipe')

        They can happen after a client disconnects abruptly (because it was
        killed, because the network is down, etc.). We assume here that if and
        when an error happens here, a disconnection will be detected shortly
        afterwards by the server which then removes this agent from the bus.
        Hence, we ignore the error; please also note that not ignoring the
        error can have an impact on code, for example, IyServer.send_msg()
        does not expect that IvyClient.send() fails and if it fails, it is
        possible that the server does not send the message to all possible
        subscribers.

        .. note:: ``ivysocket.c:SocketSendRaw()`` also ignores error, simply
           logging them.
        
        """
        try:
            self.socket.send(encode_message(msg_type, *params))
        except (socket.timeout, socket.error) as exc:
            log('[ignored] Error on socket with %r: %s', self, exc)


class IvyServer(socketserver.ThreadingTCPServer):
    """
    An Ivy server is responsible for receiving and handling the messages
    that other clients send on an Ivy bus to a given agent.

    An IvyServer has two important attributes: `usesDaemons` and
    `server_termination`.


    :ivar usesDaemons:
      whether the threads are daemonic or not.  Daemonic
      threads do not prevent python from exiting when the main thread stop,
      while non-daemonic ones do.  Default is False.  This attribute should
      be set through at `__init__()` time and should not be modified
      afterwards.

    :ivar server_termination:
      a `threading.Event` object that is set on server shutdown.  It can be
      used either to test whether the server has been stopped
      (``server_termination.isSet()``) or to wait until it is stopped
      (``server_termination.wait()``).  Application code should not try to set
      the Event directly, rather it will call `stop()` to terminate the
      server.

    :ivar port: tells on which port the TCP server awaits connection

    MT-safety
    ---------
    All public methods (not starting with an underscore ``_``) are
    MT-safe

    :group Communication on the ivybus: start, send_msg, send_direct_message,
      send_ready_message, handle_msg, stop

    :group Inspecting the ivybus: get_clients, _get_client, get_client_with_name

    :group Our own subscriptions: get_subscriptions, bind_msg, unbind_msg,
      _add_subscription, _remove_subscription, _get_fct_for_subscription

    """
    # Impl. note: acquiring/releasing the global lock in methods
    #  requiring it could be done w/ a decorator instead of repeating
    #  the acquire-try-finally-release block each time, but then we
    #  won't be compatible w/ py < 2.4 and I do not want this

    def __init__(self, agent_name, ready_msg='',
                 app_callback=void_function,
                 die_callback=void_function,
                 usesDaemons=False):
        """
        Builds a new IvyServer.  A client only needs to call `start()` on the
        newly created instances to connect to the corresponding Ivy bus and to
        start communicating with other applications.

        MT-safety: both functions `app_callback` and `die_callback` must be
        prepared to be called concurrently
        
        :Parameters:
          - `agent_name`: the client's agent name
          - `ready_msg`: a message to send to clients when they connect
          - `app_callback`: a function called each time a client connects or
            disconnects. This function is called with a single parameter
            indicating which event occurred: `IvyApplicationConnected` or
            `IvyApplicationDisconnected`.
          - `die_callback`: called when the IvyServer receives a DIE message
          - `usesDaemons`: see above.

        :see: `bind_msg()`, `start()`
        """
        self._thread = None

        # the empty string is equivalent to INADDR_ANY
        socketserver.TCPServer.__init__(self, ('', 0), IvyHandler)
        self.port = self.socket.getsockname()[1]
        #self.allow_reuse_address=True

        # private, maps (ip,port) to IvyClient!
        self._clients = {}

        # idx -> (regexp, function), see bind_msg() for details, below
        self._subscriptions = {}
        # the next index to use within the _subscriptions map.
        self._next_subst_idx = 0

        self.agent_name = agent_name
        self.ready_message = ready_msg

        # app_callback's parameter event=CONNECTED / DISCONNECTED
        self.app_callback = app_callback
        self.die_callback = die_callback
        self.direct_callback = void_function
        self.regexp_change_callback = void_function
        self.pong_callback = void_function

        # the global_lock protects: _clients, _subscriptions
        # and _next_subst_idx
        self._global_lock = threading.RLock()

        self.usesDaemons = usesDaemons
        self.server_termination = threading.Event()

        self.agent_id = agent_name + time.strftime('%Y%m%d%H%M%S') + '%05i' % random.randint(0, 99999) + str(self.port)

    def serve_forever(self):
        """
        Handle requests (calling `handle_request()`) until doomsday... or
        until `stop()` is called.

        This method is registered as the target method for the thread.
        It is also responsible for launching the UDP server in a separate
        thread, see `UDP_init_and_listen` for details.
        
        You should not need to call this method, use `start` instead.
        """
        broadcast, port = decode_ivybus(self.ivybus)
        l = lambda server=self: UDP_init_and_listen(broadcast, port, server)
        t2 = threading.Thread(target=l)
        t2.setDaemon(self.usesDaemons)
        log('Starting UDP listener')
        t2.start()

        self.socket.settimeout(0.1)
        while not self.server_termination.isSet():
            self.handle_request()
        log('TCP Ivy Server terminated')

    def start(self, ivybus=None):
        """
        Binds the server to the ivybus. The server remains connected until
        `stop` is called, or until it receives and accepts a 'die' message.

        :except IvyIllegalStateError: if the server has already been
          started
        """
        if self._thread is not None:
            error('Cannot start: IvyServer already started')
            raise IvyIllegalStateError('not running')

        self.ivybus = ivybus

        log('Starting IvyServer on port %li', self.port)
        self.server_termination.clear()
        self._thread = threading.Thread(target=self.serve_forever)
        self._thread.setDaemon(self.usesDaemons)
        self._thread.start()

    def stop(self):
        """
        Disconnects the server from the ivybus. It also sets the
        `server_termination` event.

        :except IvyIllegalStateError: if the server is not running started
        """
        if not self.isAlive():
            error('Cannot stop: not running')
            raise IvyIllegalStateError('not running')

        self._global_lock.acquire()
        try:
            for client in self._clients.values():
                try:
                    client.wave_bye()
                except socket.error:
                    pass
        finally:
            self._global_lock.release()
        self.server_termination.set()
        self._thread.join()
        self._thread = None

    def isAlive(self):
        if self._thread is None:
            return False
        return self._thread.isAlive()

    def get_clients(self):
        """
        Returns the list of the agent names of all connected clients

        :see: get_client_with_name
        """
        self._global_lock.acquire()
        try:
            return [c.agent_name for c in self._clients.values()
                    if c.status == INITIALIZED]
        finally:
            self._global_lock.release()

    def _get_client(self, ip, port, client_socket=None,
                    agent_id=None, agent_name=None):
        """
        Returns the corresponding client, and create a new one if needed.

        If agent_id is not None, the method checks whether a client with the
        same id is already registered; if it exists, the method exits by
        returning None.

        You should not need to call this, use `get_client_with_name` instead
        """
        self._global_lock.acquire()
        try:
            # if agent_id is provided, check whether it was already registered
            if agent_id and agent_id in [c.agent_id for c in self._clients.values()]:
                return None
            return self._clients.setdefault((ip, port), IvyClient(ip, port, client_socket,
                                                                  agent_id, agent_name))
        finally:
            self._global_lock.release()

    def get_client_with_name(self, name):
        """
        Returns the list of the clients registered with a given agent-name

        :see: get_clients
        """
        clients = []
        self._global_lock.acquire()
        try:
            for client in self._clients.values():
                if client.agent_name == name:
                    clients.append(client)
            return clients
        finally:
            self._global_lock.release()

    def handle_new_client(self, client):
        """
        Completes connection with the client
        TODO: maybe add a flag (while connecting) on the client, that would prevent sending msg. etc..
        as CNX. not confirmed
        """
        self.app_callback(client, IvyApplicationConnected)

    def handle_die_message(self, msg_id, from_client=None):
        """  """
        should_die = (self.die_callback(from_client, msg_id) != IVY_SHOULD_NOT_DIE)
        log('Received a die msg from: %s with id: %s -- should die=%s',
            from_client or '<unknown>', msg_id, should_die)
        if should_die:
            self.stop()
        return should_die

    def handle_direct_msg(self, client, num_id, msg):
        """
        :param client:
        :param num_id:
        :param msg:
        :return:
        """
        log('Received a direct msg from: %s with id: %s -- %s',
            client or '<unknown>', num_id, msg)
        self.direct_callback(client, num_id, msg)

    def handle_regexp_change(self, client, event, num_id, regexp):
        """
        """
        log('Regexp change: %s %s regexp %d: %s',
            client or '<unknown>',
            event == ADD_REGEXP and 'add' or 'remove',
            num_id, regexp)
        if event == ADD_REGEXP:
            event = IvyRegexpAdded
        else:
            event = IvyRegexpRemoved
        self.regexp_change_callback(client, event, num_id, regexp)

    def handle_pong(self, client, delta):
        """
        """
        self.pong_callback(client, delta)

    def remove_client(self, ip, port, trigger_application_callback=True):
        """
        Removes a registered client

        This method is responsible for calling ``server.app_callback``

        :return: the removed client, or None if no such client was found

        .. note:: NO NETWORK CLEANUP IS DONE
        """
        self._global_lock.acquire()
        try:
            try:
                removed_client = self._clients[(ip, port)]
            except KeyError:
                debug('Trying to remove a non registered client %s:%s', ip, port)
                return None
            debug('Removing client %r', removed_client)
            del self._clients[removed_client]
            if trigger_application_callback:
                self.app_callback(removed_client, IvyApplicationDisconnected)

            return removed_client
        finally:
            self._global_lock.release()

    def send_msg(self, message):
        """
        Examine the message and choose to send a message to the clients
        that subscribed to such a msg

        :return: the number of clients to which the message was sent
        """
        self._global_lock.acquire()
        count = 0
        try:
            for client in self._clients.values():
                if client.send_msg(message):
                    count += 1
        finally:
            self._global_lock.release()

        return count

    def send_direct_message(self, agent_name, num_id, msg, stop_on_first=True):
        """
        Sends a direct message to the agent named ``agent_name``.  If there
        is more than one agent with that name on the bus, parameter
        `stop_on_first` determines the behaviour.

        :Parameters:
          - `agent_name`: the name of the agent(s) to which the direct message
            should be sent.
          - `num_id`: a numerical identifier attached to the direct message
          - `msg`: the direct message itself
          - `stop_on_first`: if ``True``, the message to all agents having the
            same name will receive the message; if ``False`` the method exits
            after the first message has been sent.

        :return: ``True`` if at least one direct message was sent
        """
        self._global_lock.acquire()
        try:
            status = False
            for client in self._clients.values():
                if client.agent_name == agent_name:
                    client.send_direct_message(num_id, msg)
                    status = True
                    if stop_on_first:
                        return status
            return status
        finally:
            self._global_lock.release()

    def send_ready_message(self, client):
        """
        """
        if self.ready_message:
            client.send_msg(self.ready_message)

    def _add_subscription(self, regexp, fct):
        """
        Registers a new regexp and binds it to the supplied fct. The id
        assigned to the subscription and returned by method is **unique**
        to that subscription for the life-time of the server object: even in
        the case when a subscription is unregistered, its id will _never_
        be assigned to another subscription.

        :return: the unique id for that subscription
        """
        # explicit lock here: even if this method is private, it is
        # responsible for the uniqueness of a subscription's id, so we
        # prefer to lock it one time too much than taking the risk of
        # forgetting it (hence, the need for a reentrant lock)
        self._global_lock.acquire()
        try:
            idx = self._next_subst_idx
            self._next_subst_idx += 1
            self._subscriptions[idx] = (regexp, fct)
            return idx
        finally:
            self._global_lock.release()

    def _remove_subscription(self, idx):
        """
        Unregisters the corresponding regexp

        .. warning:: this method is not MT-safe, callers must acquire the
           global lock

        :return: the regexp that has been removed
        :except KeyError: if no such subscription can be found
        """
        return self._subscriptions.pop(idx)[0]

    def _get_fct_for_subscription(self, idx):
        """
        .. warning:: this method is not Multi-Thread-safe, callers must acquire the
           global lock
        """
        return self._subscriptions[int(idx)][1]

    def handle_msg(self, client, idx, *params):
        """
        Simply call the function bound to the subscription id `idx` with
        the supplied parameters.
        """
        self._global_lock.acquire()
        try:
            try:
                return self._get_fct_for_subscription(int(idx))(client, *params)
            except KeyError:
                # it is possible that we receive a message for a regexp that
                # was subscribed then unregistered
                warn('Asked to handle an unknown subscription: id:%r params: %r'
                     ' --ignoring', idx, params)
        finally:
            self._global_lock.release()

    def get_subscriptions(self):
        self._global_lock.acquire()
        try:
            return [(idx, s[0]) for idx, s in self._subscriptions.items()]
        finally:
            self._global_lock.release()

    def bind_direct_msg(self, on_direct_msg_fct):
        """
        """
        self.direct_callback = on_direct_msg_fct

    def bind_regexp_change(self, on_regexp_change_callback):
        """
        """
        self.regexp_change_callback = on_regexp_change_callback

    def bind_pong(self, on_pong_callback):
        """
        """
        self.pong_callback = on_pong_callback

    def bind_msg(self, on_msg_fct, regexp):
        """
        Registers a new subscriptions, by binding a regexp to a function, so
        that this function is called whenever a message matching the regexp
        is received.

        :Parameters:
          - `on_msg_fct`: a function accepting as many parameters as there is
            groups in the regexp. For example:

            - the regexp ``'^hello .*'`` corresponds to a function called w/ no
              parameter,
            - ``'^hello (.*)'``: one parameter,
            - ``'^hello=([^ ]*) from=([^ ]*)'``: two parameters

          - `regexp`: (string) a regular expression

        :return: the binding's id, which can be used to unregister the binding
          with `unbind_msg()`
        """
        self._global_lock.acquire()
        idx = self._add_subscription(regexp, on_msg_fct)
        try:
            for client in self._clients.values():
                client.send_new_subscription(idx, regexp)
        finally:
            self._global_lock.release()
        return idx

    def unbind_msg(self, num_id):
        """
        Unbinds a subscription

        :param num_id: the binding's id, as returned by `bind_msg()`

        :return: the regexp corresponding to the unsubscribed binding
        :except KeyError: if no such subscription can be found
        """
        self._global_lock.acquire()
        try:
            regexp = self._remove_subscription(num_id)  # KeyError
            # notify others that we have no interest anymore in this regexp
            for client in self._clients.values():
                client.remove_subscription(num_id)
        finally:
            self._global_lock.release()
        return regexp


class IvyHandler(socketserver.StreamRequestHandler):
    """
    An IvyHandler is associated to one IvyClient connected to our server.

    It runs into a dedicate thread as long as the remote client is connected
    to us.

    It is in charge of examining all messages that are received and to
    take any appropriate actions.

    Implementation note: the IvyServer is accessible in ``self.server``
    """

    def handle(self):
        """
        """
        # self.request is the socket object
        # self.server is the IvyServer

        bufsize = 1024
        client_socket = self.request
        ip = self.client_address[0]
        port = self.client_address[1]

        trace('New IvyHandler for %s:%s, socket %r', ip, port, client_socket)

        client = self.server._get_client(ip, port, client_socket)
        debug('Got a request from ip=%s port=%s', ip, port)

        # First, send our initial subscriptions
        client_socket.send(encode_message(START_INIT, self.server.port,
                                          self.server.agent_name))
        for idx, subscr in self.server.get_subscriptions():
            client_socket.send(encode_message(ADD_REGEXP, idx, subscr))
        client_socket.send(encode_message(END_REGEXP, 0))

        while self.server.isAlive():
            try:
                if PY2:
                    msgs = client_socket.recv(bufsize)
                else:
                    msgs = client_socket.recv(bufsize).decode('UTF-8')
            except socket.timeout:
                trace('timeout on socket bound to client %r', client)
                continue
            except socket.error as e:
                log('Error on socket with %r: %s', client, e)
                self.server.remove_client(ip, port)
                break  # the server will close the TCP connection

            if not msgs:
                # client is not connected anymore
                log('Lost connection with %r', client)
                self.server.remove_client(ip, port)
                break  # the server will close the TCP connection

            # Sometimes the message is not fully read on the first try,
            # so we insist to get the final newline
            if msgs[-1:] != '\n':

                # w/ the following idioms (also replicated a second time below)
                # we make sure that we wait until we get a message containing
                # the final newline, or if the server is terminated we stop
                # handling the request
                while self.server.isAlive():
                    try:
                        if PY2:
                            _msg = client_socket.recv(bufsize)
                        else:
                            _msg = client_socket.recv(bufsize).decode('UTF-8')
                        break
                    except socket.timeout:
                        continue
                if not self.server.isAlive():
                    break

                msgs += _msg
                while _msg[-1:] != '\n' and self.server.isAlive():

                    while self.server.isAlive():
                        try:
                            if PY2:
                                _msg = client_socket.recv(bufsize)
                            else:
                                _msg = client_socket.recv(bufsize).decode('UTF-8')
                            break
                        except socket.timeout:
                            continue

                    msgs += _msg

                if not self.server.isAlive():
                    break

            debug('Got a request from ip=%s port=%s: %r', ip, port, msgs)

            msgs = msgs[:-1]

            msgs = msgs.split('\n')
            for msg in msgs:
                keep_connection_alive = self.process_ivymessage(msg, client)
                if not keep_connection_alive:
                    self.server.remove_client(ip, port)
                    break
        log('Closing connection to client %r', client)

    def process_ivymessage(self, msg, client):
        """
        Examines the message (after passing it through the `decode_msg()`
        filter) and takes the appropriate actions depending on the message
        types.  Please refer to the document `The Ivy Architecture and
        Protocol <http://www.tls.cena.fr/products/ivy/documentation>`_ and to
        the python code for further details.
        
        
        :Parameters:
          - `msg`: (should not include a newline at end)

        :return: ``False`` if the connection should be terminated, ``True``
          otherwise
        
        """
        # cf. static void Receive() in ivy.c
        try:
            msg_id, num_id, params = decode_msg(msg)
        except IvyMalformedMessage:
            warn('Received an incorrect message: %r from: %r', msg, client)
            # TODO: send back an error message
            return True

        debug('Got: msg_id: %r, num_id: %r, params: %r',
              msg_id, num_id, params)

        err_msg = ''
        try:
            if msg_id == BYE:
                # num_id: not meaningful. No parameter.
                log('%s waves bye-bye: disconnecting', client)
                return False

            elif msg_id == ADD_REGEXP:
                # num_id=id for the regexp, one parameter: the regexp
                err_msg = 'Client %r was not properly initialized' % client
                log('%s sending a new subscription id:%r regexp:%r ',
                    client, num_id, params)
                client.add_regexp(num_id, params)
                self.server.handle_regexp_change(client, ADD_REGEXP,
                                                 num_id, params)
                # TODO: handle errors (e.g. 2 subscriptions w/ the same id)

            elif msg_id == DEL_REGEXP:
                # num_id=id for the regexp to removed, no parameter
                err_msg = 'Client %r was not properly initialized' % client
                log('%s removing subscription id:%r', client, num_id)
                try:
                    regexp = client.remove_regexp(num_id)
                    self.server.handle_regexp_change(client, DEL_REGEXP,
                                                     num_id, regexp)
                except KeyError:
                    # TODO: what else?
                    warn('%s tried to remove a non-registered subscription w/ id:%r', client, num_id)

            elif msg_id == MSG:
                # num_id: regexp_id, parameters: the substrings captured by
                # the regexp
                log('From %s: (regexp=%s) %r', client, num_id, params)
                self.server.handle_msg(client, num_id, *params)

            elif msg_id == ERROR:
                # num_id: not meaningful, parameter=error msg
                warn('Client %r sent a protocol error: %s', client, params)
                # TODO: send BYE and close connection, as in ivy.c?

            elif msg_id == START_INIT:
                # num_id: tcp port number, parameter: the client's agentname
                err_msg = 'Client %r sent the initial subscription more than once' % client
                client.start_init(params)
                log('%s connected from %r', params, client)

            elif msg_id == END_INIT:
                # num_id: not meaningful. No parameter.
                client.end_init()
                # app. callback
                self.server.handle_new_client(client)
                # send ready message
                self.server.send_ready_message(client)

            elif msg_id == DIE:
                # num_id: not meaningful. No parameter.
                self.server.handle_die_message(num_id, client)

            elif msg_id == DIRECT_MSG:
                # num_id: not meaningful. 
                log('Client %r sent us a direct msg num_id:%s msg:%r',
                    client, num_id, params)
                self.server.handle_direct_msg(client, num_id, params)

            elif msg_id == PING:
                debug('Received PING w/ id: %i from %r', num_id, client)
                client._send(PONG, num_id)

            elif msg_id == PONG:
                debug('Received PONG from %r (id: %i)', client, num_id)
                delta = client.get_next_ping_delta()
                if delta is None:
                    warn('Client %r sent a unsollicited PONG msg', client)
                else:
                    self.server.handle_pong(client, delta)

            else:
                warn('Unhandled msg from %r: %r', client, msg)

        except IvyIllegalStateError:
            raise IvyProtocolError(err_msg)

        return True


class IvyTimer(threading.Thread):
    """
    An IvyTimer object is responsible for calling a function regularly.  It is
    bound to an IvyServer and stops when its server stops.

    Interacting with a timer object
    -------------------------------

    - Each timer gets a unique id, stored in the attribute ``id``. Note that
      a dead timer's id can be reassigned to another one (a dead timer is a
      timer that has been stopped)

    - To start a timer, simply call its method ``start()``
    
    - To modify a timer's delay: simply assign ``timer.delay``, the
      modification will be taken into account after the next tick. The delay
      should be given in milliseconds.

    - to stop a timer, assign ``timer.abort`` to ``True``, the timer will stop
      at the next tick (the callback won't be called)

    MT-safety
    ---------
    **Please note:** ``start()`` starts a new thread; if the same function is
    given as the callback to different timers, that function should be
    prepared to be called concurrently.  Specifically, if the callback
    accesses shared variables, they should be protected against concurrency
    problems (using locks e.g.).
    """

    def __init__(self, server, nbticks, delay, callback):
        """
        Creates a new timer.  After creation, call the timer's ``start()``
        method to activate it.
        
        :Parameters:
          - `server`: the `IvyServer` related to this timer --when the server
            stops, so does the timer.
          - `nbticks`: the number of repetition to make. ``0`` (zero) means:
            endless loop
          - `delay`: the delay, in milliseconds, between two ticks
          - `callback`: a function called at each tick. This function is
            called with one parameter, the timer itself
            
        """
        threading.Thread.__init__(self)
        self.server = server
        self.nbticks = nbticks
        self.delay = delay       # milliseconds
        self.callback = callback
        self.abort = False
        self.id = id(self)
        self.setDaemon(server.usesDaemons)

    def run(self):
        ticks = -1
        while self.server.isAlive() and not self.abort and ticks < self.nbticks:

            if self.nbticks:  # 0 means: endless
                ticks += 1
            self.callback(self)
            time.sleep(self.delay / 1000.0)
        log('IvyTimer %s terminated', self.id)


if __name__ == '__main__':
    s = IvyServer(agent_name='TEST_APP',
                  ready_msg='[Youkou]')
    s.start()

    def dflt_fct(*args):
        log('DFLT_FCT: Received: %r', args)

    time.sleep(1)

    for regexp in ('^test .*', '^test2 (.*)$', 'test3 ([^-]*)-?(.*)', '(.*)'):
        s.bind_msg(dflt_fct, regexp)
    time.sleep(1)

    s.send_msg('glop pas glop -et paf')
    s.send_msg('glosp pas glop -et paf')
    time.sleep(1000000)
