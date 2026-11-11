"""Fast DDS profiles for a remote RViz over loopback SSH forwards."""
from pathlib import Path

PORTS = {
    'move_group': 42161,
    'robot_state_publisher': 42162,
    'static_transform_publisher': 42163,
    'ros2_control_node': 42164,
}

UDP = (
    '<transport_descriptor><transport_id>loopback_udp</transport_id><type>UDPv4</type>'
    '<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList></transport_descriptor>'
)


def profile_xml(listen_port=None):
    server = listen_port is not None
    tcp_listen = f'<listening_ports><port>{listen_port}</port></listening_ports>' if server else ''
    udp_id = '<transport_id>loopback_udp</transport_id>' if server else ''
    peers = ('<locator><udpv4><address>127.0.0.1</address></udpv4></locator>'
             if server else ''.join(
                 f'<locator><tcpv4><address>127.0.0.1</address><physical_port>{port}</physical_port>'
                 '</tcpv4></locator>' for port in PORTS.values()))
    return f"""<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><profiles>
    <transport_descriptors>
      {UDP if server else ''}
      <transport_descriptor><transport_id>rviz_tcp</transport_id><type>TCPv4</type>
        <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
        {tcp_listen}<enable_tcp_nodelay>true</enable_tcp_nodelay>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="coarm7_remote_rviz" is_default_profile="true"><rtps>
      <userTransports>{udp_id}<transport_id>rviz_tcp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin><initialPeersList>{peers}</initialPeersList></builtin>
    </rtps></participant>
    </profiles></dds>
"""


def profile_environment(path, listen_port=None):
    path = Path(path)
    path.write_text(profile_xml(listen_port))
    return {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'ROS_LOCALHOST_ONLY': '0',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(path),
    }


def server_environment(directory):
    return {name: profile_environment(directory / f'{name}.xml', port)
            for name, port in PORTS.items()}
