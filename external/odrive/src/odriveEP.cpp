#include "odrive/odrive.hpp"
#include "odrive/odriveEP.hpp"

using namespace odrive;

ODriveEP::ODriveEP() {devHandle = NULL;}

ODriveEP::~ODriveEP() {
  if (devHandle) {
    int res = libusb_release_interface(devHandle, 0);
    if (res != LIBUSB_SUCCESS) {
       std::cerr << "Error calling libusb_release_interface on odrive `" << "`: " << res << " - " << libusb_error_name(res) << std::endl;
    }
    libusb_close(devHandle);
    devHandle = NULL;
  }
}

/**
 * Append short data to data buffer
 * @param buf data buffer
 * @param value data to append
 */
void ODriveEP::appendShortToCommBuffer(commBuffer& buf, const short value) {
  buf.push_back((value >> 0) & 0xFF);
  buf.push_back((value >> 8) & 0xFF);
}

/**
 * Append int data to data buffer
 * @param buf data buffer
 * @param value data to append
 */
void ODriveEP::appendIntToCommBuffer(commBuffer& buf, const int value) {
  buf.push_back((value >> 0) & 0xFF);
  buf.push_back((value >> 8) & 0xFF);
  buf.push_back((value >> 16) & 0xFF);
  buf.push_back((value >> 24) & 0xFF);
}

/**
 *  Decode odrive packet
 *  @param buf data buffer
 *  @param seq_no packet sequence number
 *  @return data buffer
 */
commBuffer ODriveEP::decodeODrivePacket(commBuffer& buf, short& seq_no) {
  commBuffer payload;
//   std::cout << "decode buf size = " << buf.size() << std::endl;
  memcpy(&seq_no, &buf[0], sizeof(short));
  seq_no &= 0x7fff;
  for (commBuffer::size_type i = 2; i < buf.size(); ++i) {
    payload.push_back(buf[i]);
  }
  return payload;
}

/**
 * Create data packet
 * @param seq_no next sequence number
 * @param endpoint_id USB endpoint ID
 * @param response_size maximum data length to be read
 * @param read append request address
 * @param address desctination address
 * @param input data buffer to send
 * @return data buffer read
 */
commBuffer ODriveEP::createODrivePacket(short seq_no, int endpoint_id,
                short response_size, bool read, int address, const commBuffer& input) {
  commBuffer packet;
  short crc = 0;

  if ((endpoint_id & 0x7fff) == 0) {
    crc = ODRIVE_PROTOCOL_VERION;
  } else {
    crc = ODRIVE_DEFAULT_CRC_VALUE;
  }
  appendShortToCommBuffer(packet, seq_no);
  appendShortToCommBuffer(packet, endpoint_id);
  appendShortToCommBuffer(packet, response_size);
  if (read) appendIntToCommBuffer(packet, address);
  for (uint8_t b : input) packet.push_back(b);
  appendShortToCommBuffer(packet, crc);
  return packet;
}

/**
 *  Read value from ODrive
 *  @param id odrive ID
 *  @param value Data read
 *  @return 0 on success
 */
template<typename T>
int ODriveEP::getData(int id, T& value) {
    commBuffer tx, rx;
    int rx_size;

    int res = endpointRequest(id, rx, rx_size, tx, 1, sizeof(value));
    if (res != 0) {
        return res;
    }
    if (rx.size() > 0) memcpy(&value, &rx[0], sizeof(value));
	else std::cout << "rx size = " << rx.size() << std::endl;
    return 0;
}


/**
 *  Request function to ODrive
 *  @param id odrive ID
 *  @return 0 on success
 */
int ODriveEP::execFunc(int endpoint_id) {
    commBuffer tx, rx;
    int rx_length;
    return endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
}

/**
 *  Write value to Odrive
 *  @param id odrive ID
 *  @param value Data to be written
 *  @return 0 on success
 *
 */
template<typename T>
int ODriveEP::setData(int endpoint_id, const T& value) {
    commBuffer tx, rx;
    int rx_length;
    for (int i = 0; i < sizeof(value); i++){
       tx.push_back(((unsigned char*)&value)[i]);
    }
    return endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
}

/**
 * Request endpoint
 * @param handle USB device handler
 * @param endpoint_id odrive ID
 * @param received_payload receive buffer
 * @param received_length receive length
 * @param payload data read
 * @param ack request acknowledge
 * @param length data length
 * @param read send read address
 * @param address read address
 * @return LIBUSB_SUCCESS on success
 */
int ODriveEP::endpointRequest(int endpoint_id, commBuffer& received_payload, int& received_length, 
                              commBuffer payload, bool ack, int length, bool read, int address) {
  commBuffer send_buffer, receive_buffer;
  unsigned char receive_bytes[ODRIVE_MAX_RESULT_LENGTH] = { 0 };
  int sent_bytes = 0;
  int received_bytes = 0;
  short received_seq_no = 0;
    
  ep_lock.lock();
  // Prepare sequence number
  if (ack) {
    endpoint_id |= 0x8000;
  }
  outboundSeqNo = (outboundSeqNo + 1) & 0x7fff;
  outboundSeqNo |= LIBUSB_ENDPOINT_IN; 
  short seq_no = outboundSeqNo;

  // Create request packet
  commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, read, address, payload);
  // Transfer paket to target
//   std::cout << "ep req: bulk transfer with bytes to send " << sent_bytes << std::endl;
  int result = libusb_bulk_transfer(devHandle, ODRIVE_OUT_EP, packet.data(), packet.size(), &sent_bytes, 0);
//   std::cout << "transfer done: nof bytes " << sent_bytes << std::endl;
  if (result != LIBUSB_SUCCESS) {
    std::cerr << "Error in transfering data to USB" << std::endl;
    ep_lock.unlock();
    return result;
  } else if (packet.size() != sent_bytes) {
    std::cerr << "Error in transfering data to USB, not all data transferred" << std::endl;
  }

  // Get responce
  if (ack) {
// 	std::cout << "response: bulk transfer with bytes to send " << sent_bytes << std::endl;
    result = libusb_bulk_transfer(devHandle, ODRIVE_IN_EP, receive_bytes, ODRIVE_MAX_BYTES_TO_RECEIVE, &received_bytes, ODRIVE_TIMEOUT);
//     std::cout << "transfer done: nof bytes " << sent_bytes << std::endl;
    if (result != LIBUSB_SUCCESS) {
      std::cerr << "Error in reading data from USB, res = " << result <<  std::endl;
      ep_lock.unlock();
      return result;
    }

    // Push received data to buffer
    for (int i = 0; i < received_bytes; i++) {
      receive_buffer.push_back(receive_bytes[i]);
    }
    received_payload = decodeODrivePacket(receive_buffer, received_seq_no);
    if (received_seq_no != seq_no) {
      std::cerr << "Error: received data out of order, received_seq_no = " << received_seq_no << ", seq_no = " << seq_no << std::endl;
    }
    received_length = received_payload.size();
  }
  ep_lock.unlock();
  return LIBUSB_SUCCESS;
}

template int ODriveEP::getData(int, bool&);
template int ODriveEP::getData(int, short&);
template int ODriveEP::getData(int, int&);
template int ODriveEP::getData(int, float&);
template int ODriveEP::getData(int, uint8_t&);
template int ODriveEP::getData(int, uint16_t&);
template int ODriveEP::getData(int, uint32_t&);
template int ODriveEP::getData(int, uint64_t&);

template int ODriveEP::setData(int, const bool&);
template int ODriveEP::setData(int, const short&);
template int ODriveEP::setData(int, const int&);
template int ODriveEP::setData(int, const float&);
template int ODriveEP::setData(int, const uint8_t&);
template int ODriveEP::setData(int, const uint16_t&);
template int ODriveEP::setData(int, const uint32_t&);
template int ODriveEP::setData(int, const uint64_t&);

