package com.dronegcs.mavlink.core.connection;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import com.dronegcs.mavlink.is.connection.MavLinkConnection;
import com.dronegcs.mavlink.is.connection.MavLinkConnectionTypes;

/**
 * Provides support for com.dronegcs.mavlink.is.mavlink connection via udp.
 */
public abstract class UdpConnection extends MavLinkConnection {

	private DatagramSocket socket;
	private int serverPort;

	private int hostPort;
	private InetAddress hostAdd;

	private void getUdpStream() throws IOException {
		socket = new DatagramSocket(serverPort);
		socket.setBroadcast(true);
		socket.setReuseAddress(true);
	}

	@Override
	public final boolean closeConnection() throws IOException {
		if (socket != null) {
			socket.close();
			return false;
		}
		return true;
	}

	@Override
	public final boolean openConnection() throws IOException {
		getUdpStream();
		return true;
	}

	@Override
	public final void sendBuffer(byte[] buffer) throws IOException {
		try {
			if (hostAdd != null) { // We can't send to our sister until they
				// have connected to us
				DatagramPacket packet = new DatagramPacket(buffer, buffer.length, hostAdd, hostPort);
				socket.send(packet);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public final int readDataBlock(byte[] readData) throws IOException {
		DatagramPacket packet = new DatagramPacket(readData, readData.length);
		socket.receive(packet);
		hostAdd = packet.getAddress();
		hostPort = packet.getPort();
		return packet.getLength();
	}

	@Override
	public final void loadPreferences() {
		serverPort = loadServerPort();
	}

	@Override
	public final int getConnectionType() {
		return MavLinkConnectionTypes.MAVLINK_CONNECTION_UDP;
	}

	protected abstract int loadServerPort();
}
