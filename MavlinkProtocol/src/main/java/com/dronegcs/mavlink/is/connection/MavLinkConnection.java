package com.dronegcs.mavlink.is.connection;

import java.io.IOException;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import javax.validation.constraints.NotNull;

import com.dronegcs.mavlink.core.connection.USBConnection;
import com.generic_tools.logger.Logger;
import gnu.io.PortInUseException;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.DroneEventsType;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_heartbeat;
import com.dronegcs.mavlink.is.protocol.msgparser.Parser;

/**
 * Base for mavlink connection implementations.
 */
public abstract class MavLinkConnection {

	private final org.slf4j.Logger LOGGER = LoggerFactory.getLogger(USBConnection.class);

//	@SuppressWarnings("unused")
//	private static final String TAG = MavLinkConnection.class.getSimpleName();

	@Autowired @NotNull(message = "Internal Error: Failed to get drone")
	private Drone drone;

	@Autowired @NotNull(message = "Internal Error: Failed to get logger")
	private Logger logger;

	protected ConnectionStatistics connectionStatistics;

	/*
	 * MavLink connection states
	 */
	private static final int MAVLINK_DISCONNECTED = 0;
	private static final int MAVLINK_CONNECTING = 1;
	private static final int MAVLINK_CONNECTED = 2;

	/**
	 * Size of the buffer used to read messages from the com.dronegcs.mavlink.is.mavlink connection.
	 */
	private static final int READ_BUFFER_SIZE = 4096*8;//4;

	/**
	 * Maximum possible sequence number for a packet.
	 */
	private static final int MAX_PACKET_SEQUENCE = 255;

	/**
	 * Set of listeners subscribed to this com.dronegcs.mavlink.is.mavlink connection. We're using a
	 * ConcurrentSkipListSet because the object will be accessed from multiple
	 * threads concurrently.
	 */
	private final ConcurrentHashMap<String, MavLinkConnectionListener> mListeners = new ConcurrentHashMap<>();

	/**
	 * Set of listeners subscribed to this com.dronegcs.mavlink.is.mavlink connection statistics. We're using a
	 */
	private final ConcurrentHashMap<String, MavLinkConnectionStatisticsListener> mConnectionStatisticsListeners = new ConcurrentHashMap<>();

	/**
	 * Queue the set of packets to send via the com.dronegcs.mavlink.is.mavlink connection. A thread
	 * will be blocking on it until there's element(s) available to send.
	 */
	private final LinkedBlockingQueue<MAVLinkPacket> mPacketsToSend = new LinkedBlockingQueue<MAVLinkPacket>();

	private final AtomicInteger mConnectionStatus = new AtomicInteger(MAVLINK_DISCONNECTED);

	/**
	 * Listen for incoming data on the com.dronegcs.mavlink.is.mavlink connection.
	 */
	private final Runnable mConnectingTask = new Runnable() {

		@Override
		public void run() {
			LOGGER.debug("Starting Connection Tasks Thread (Reader)");
			Thread sendingThread = null; 

			// Load the connection specific preferences
			loadPreferences();

			try {
				// Open the connection
				if (!openConnection()) {
					logger.LogErrorMessege("Failed to open connection");
					LOGGER.error("Failed to open connection");
					drone.notifyDroneEvent(DroneEventsType.DISCONNECTED);
					return;
				}
				
				mConnectionStatus.set(MAVLINK_CONNECTED);
				reportConnect();

				logger.LogGeneralMessege("Mavlink connected");
				LOGGER.debug("Mavlink connected");

				// Launch the 'Sending' thread
				sendingThread = new Thread(mSendingTask, "MavLinkConnection-Sending Thread");
				sendingThread.start();

				logger.LogGeneralMessege("Preparing Mavlink parser");
				LOGGER.debug("Preparing Mavlink parser");
				final Parser parser = new Parser();

				logger.LogGeneralMessege("Reset connection statistics");
				LOGGER.debug("Reset connection statistics");
				parser.stats.mavlinkResetStats();

				final byte[] readBuffer = new byte[READ_BUFFER_SIZE];

				drone.notifyDroneEvent(DroneEventsType.CONNECTED);

				// Statistics
				long packetsSinceLastRead = 0;
				long lastReadTimestamp = System.currentTimeMillis();

				while (mConnectionStatus.get() == MAVLINK_CONNECTED) {
					int bufferSize = readDataBlock(readBuffer);
					int msg = handleData(parser, bufferSize, readBuffer);

					// Statistics
					connectionStatistics.setReceivedPackets(connectionStatistics.getReceivedPackets() + msg);
					packetsSinceLastRead += msg;
					if (System.currentTimeMillis() - lastReadTimestamp > 1000) {
						connectionStatistics.setReceivedPacketsPerSecond((long) ((1.0 * packetsSinceLastRead) /
								((System.currentTimeMillis() - lastReadTimestamp) / 1000)));
						lastReadTimestamp = System.currentTimeMillis();
						packetsSinceLastRead = 0;
					}
				}

				// When connection is closed we will wait for the sending thread
				sendingThread.join();

				logger.LogGeneralMessege("Mavlink disconnected");
				LOGGER.debug("Mavlink disconnected");
			} 
			catch (IOException e) {
				// Ignore errors while shutting down
				if (mConnectionStatus.get() != MAVLINK_DISCONNECTED) {
					reportComError(e.getMessage());
				}
			}
			catch (PortInUseException e) {
				logger.LogErrorMessege("Port in use");
				LOGGER.error("Port in use");
				disconnect();
			}
			catch (Exception e) {
				logger.LogErrorMessege("Mavlink disconnected unexpectedly");
				LOGGER.error("Mavlink disconnected unexpectedly", e);
				disconnect();
			}
			finally {
				logger.LogDesignedMessege("Connection Thread finished");
				drone.notifyDroneEvent(DroneEventsType.DISCONNECTED);
			}
		}

		private int handleData(Parser parser, int bufferSize, byte[] buffer) {
			if (bufferSize < 1) {
				return 0;
			}

			int messages = 0;
			for (int i = 0; i < bufferSize; i++) {
//				System.err.println("Index is " + i);
				MAVLinkPacket receivedPacket = parser.mavlink_parse_char(buffer[i] & 0x00ff);
				if (receivedPacket != null) {
					MAVLinkMessage msg = receivedPacket.unpack();
					LOGGER.trace("[RCV] {}", msg);
//					System.err.println("[RCV] " + msg);
					reportReceivedMessage(msg);
					messages++;
				}
				connectionStatistics.setReceivedErrorPackets(parser.stats.crcErrorCount);
				connectionStatistics.setLostPackets(parser.stats.lostPacketCount);
			}

			return messages;
		}
	};

	/**
	 * Blocks until there's packet(s) to send, then dispatch them.
	 */
	private final Runnable mSendingTask = new Runnable() {
		@Override
		public void run() {
			LOGGER.debug("Starting Sending Tasks Thread (Writer)");
			int msgSeqNumber = 0;

			try {
				// Statistics
				long packetsSinceLastWrite = 0;
				long lastWriteTimestamp = System.currentTimeMillis();

				while (mConnectionStatus.get() == MAVLINK_CONNECTED) {
					final MAVLinkPacket packet = mPacketsToSend.take();

					MAVLinkMessage mavLinkMessage = packet.unpack();
					if (mavLinkMessage == null) {
						connectionStatistics.setTransmittedErrorPackets(connectionStatistics.getTransmittedErrorPackets() + 1);
						LOGGER.error("Failed to unpack packet '{}'", packet);
						continue;
					}

					if (mavLinkMessage.msgid != msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT) {
//						System.err.println("[SND] " + mavLinkMessage.toString());
						LOGGER.trace("[SND] {}", mavLinkMessage.toString());
						String log_entry = Logger.generateDesignedMessege(packet.unpack().toString(), Logger.Type.OUTGOING, false);
						logger.LogDesignedMessege(log_entry);
					}
					packet.seq = msgSeqNumber;
					byte[] buffer = packet.encodePacket();

					try {
						sendBuffer(buffer);

						// Statistics
						connectionStatistics.setTransmittedPackets(connectionStatistics.getTransmittedPackets() + 1);
						packetsSinceLastWrite++;
						if (System.currentTimeMillis() - lastWriteTimestamp > 1000) {
							connectionStatistics.setTransmittedPacketsPerSecond((long) ((1.0 * packetsSinceLastWrite) /
									((System.currentTimeMillis() - lastWriteTimestamp) / 1000)));
							lastWriteTimestamp = System.currentTimeMillis();
							packetsSinceLastWrite = 0;
						}
					} catch (IOException e) {
						reportComError(e.getMessage());
					}

					msgSeqNumber = (msgSeqNumber + 1) % (MAX_PACKET_SEQUENCE + 1);
				}
				logger.LogGeneralMessege("Mavlink sending thread is shutting down");
				LOGGER.debug("Mavlink sending thread is shutting down");
			} catch (InterruptedException e) {
				logger.LogErrorMessege("Interrupted exception:");
				logger.LogErrorMessege(e.getMessage());
				LOGGER.error("Interrupted exception:", e);
			} catch (Exception e) {
				LOGGER.error("exception:", e);
				disconnect();
			}
		}
	};
	
	private Thread mConnectingThread;

	private ScheduledFuture scheduledMonitorFuture;
	private final Runnable monitorTask = () -> {
		if (mConnectionStatisticsListeners == null || mConnectionStatisticsListeners.isEmpty()) {
			LOGGER.debug("No statistics listener");
			return;
		}

		LOGGER.trace("Sending connections statistics");
		for (MavLinkConnectionStatisticsListener listeners : mConnectionStatisticsListeners.values()) {
			listeners.onConnectionStatistics(connectionStatistics);
		}
	};
	
	/**
	 * Establish a mavlink connection. If the connection is successful, it will
	 * be reported through the MavLinkConnectionListener interface.
	 */
	public void connect() {
		connectionStatistics = new ConnectionStatistics();
		if (mConnectionStatus.compareAndSet(MAVLINK_DISCONNECTED, MAVLINK_CONNECTING)) {
			scheduledMonitorFuture = Executors.newScheduledThreadPool(1).scheduleAtFixedRate(monitorTask, 0, 1, TimeUnit.SECONDS);
			mConnectingThread = new Thread(mConnectingTask, "MavLinkConnection-Connecting Thread");
			mConnectingThread.start();
		}
	}

	/**
	 * Disconnect a com.dronegcs.mavlink.is.mavlink connection. If the operation is successful, it will
	 * be reported through the MavLinkConnectionListener interface.
	 */
	public void disconnect() {
		if (mConnectionStatus.compareAndSet(MAVLINK_CONNECTED, MAVLINK_DISCONNECTED)) {
			try {
				mConnectingThread.join();
				if (scheduledMonitorFuture != null) {
					scheduledMonitorFuture.cancel(false);
					scheduledMonitorFuture = null;
				}
				closeConnection();
			}
			catch (Exception e) {
				logger.LogErrorMessege("Unexpected exception during Mavlink disconnection: " + e.getMessage());
				LOGGER.error("Unexpected exception during Mavlink disconnection", e);
			}
			reportDisconnect();
		}
	}

	private int getConnectionStatus() {
		return mConnectionStatus.get();
	}

	public void sendMavPacket(MAVLinkPacket packet) {
		if (!mPacketsToSend.offer(packet)) {
			logger.LogErrorMessege("Unable to send mavlink packet. Packet queue is full!");
			LOGGER.error("Unable to send mavlink packet. Packet queue is full!");
		}
	}

	/**
	 * Adds a listener to the mavlink connection.
	 * 
	 * @param listener
	 * @param tag
	 *            Listener tag
	 */
	public void addMavLinkConnectionListener(String tag, MavLinkConnectionListener listener) {
		mListeners.put(tag, listener);

		if (getConnectionStatus() == MAVLINK_CONNECTED) {
			listener.onConnect();
		}
	}

	/**
	 * Adds a listener to the mavlink connection statistics.
	 *
	 * @param listener
	 * @param tag
	 *            Listener tag
	 */
	public void addMavLinkConnectionStatisticsListener(String tag, MavLinkConnectionStatisticsListener listener) {
		mConnectionStatisticsListeners.put(tag, listener);
	}

	/**
	 * Removes the specified listener.
	 * 
	 * @param tag
	 *            Listener tag
	 */
	public void removeMavLinkConnectionListener(String tag) {
		mListeners.remove(tag);
	}

	protected abstract boolean openConnection() throws Exception;

	protected abstract int readDataBlock(byte[] buffer) throws IOException;

	protected abstract void sendBuffer(byte[] buffer) throws IOException;

	protected abstract boolean closeConnection() throws IOException;

	protected abstract void loadPreferences();

	/**
	 * @return The type of this mavlink connection.
	 */
	protected abstract int getConnectionType();

	/**
	 * Utility method to notify the mavlink listeners about communication
	 * errors.
	 * 
	 * @param errMsg
	 */
	private void reportComError(String errMsg) {
		if (mListeners.isEmpty())
			return;

		for (MavLinkConnectionListener listener : mListeners.values()) {
			listener.onComError(errMsg);
		}
	}

	/**
	 * Utility method to notify the mavlink listeners about a successful
	 * connection.
	 */
	private void reportConnect() {
		for (MavLinkConnectionListener listener : mListeners.values()) {
			listener.onConnect();
		}
	}

	/**
	 * Utility method to notify the mavlink listeners about a connection
	 * disconnect.
	 */
	private void reportDisconnect() {
		if (mListeners.isEmpty())
			return;

		for (MavLinkConnectionListener listener : mListeners.values()) {
			listener.onDisconnect();
		}
	}

	/**
	 * Utility method to notify the mavlink listeners about received messages.
	 * 
	 * @param msg
	 *            received mavlink message
	 */
	private void reportReceivedMessage(MAVLinkMessage msg) {
		if (mListeners.isEmpty())
			return;

		for (MavLinkConnectionListener listener : mListeners.values()) {
			listener.onReceiveMessage(msg);
		}
	}
	
	public boolean isConnected() {
		return mConnectionStatus.get() == MAVLINK_CONNECTED;
	}

}
