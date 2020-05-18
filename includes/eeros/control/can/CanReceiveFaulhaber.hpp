#ifndef ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
#define ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen.h>
#include <canopen-drv.h>
#include <map>

using namespace eeros::control;
using namespace eeros::logger;

namespace eeros {
namespace control {

class CanReceiveFaulhaber: public Block {
 public:
  CanReceiveFaulhaber(int socket, std::initializer_list<uint8_t> node, std::initializer_list<uint8_t> functionCode) 
      :  socket(socket), nodes(node), functionCodes(functionCode), log('C') {
    log.info() << "CAN rec block constructed, " << node.size() << " nodes present";
    for (int i = 0; i < node.size(); i++) {
      std::map<uint32_t, PdoUserData> m1;
      outStatus.push_back(new Output<uint16_t>());
      outPos.push_back(new Output<double>());
      outPosScale.push_back(1.0);
      outVel.push_back(new Output<double>());
      outVelScale.push_back(1.0);
      outWarning.push_back(new Output<uint32_t>());
      for(int j = 0; j < functionCode.size(); j++) {
        m1[functionCodes[j]] = {functionCodes[j],static_cast<int16_t>(0),static_cast<int32_t>(0)};
      }
      nodePdoData.push_back(m1);
    }
    std::cout << "function code size: " << functionCode.size() << std::endl;
    
    for(int j = 0; j < nodePdoData.size(); j++){
      std::cout << "map"<< j << " of node " << nodes[j] << std::endl;
      for(auto p = nodePdoData[j].begin(); p != nodePdoData[j].end(); ++p){
        std::cout << p->first << "\t" << (p->second.functionCode)+0x30-48 << "\t" << p->second.drvCtrl << "\t" << p->second.userData << std::endl;
      }
    }
  }
          
  virtual ~CanReceiveFaulhaber() {
    nodes.clear();
    outStatus.clear();
    outPos.clear();
    outVel.clear();
    functionCodes.clear();
    nodePdoData.clear();
    outWarning.clear(); 
  }

  virtual void run() {
    if (enabled) {
      for (int i = 0; i < 2; i++) {
        uint64_t ts = eeros::System::getTimeNs();
        int readLen = 0;
        canopen_frame_t readFrame;
        readLen = canopen_read_frame(socket, &readFrame);
        if (readLen == EAGAIN || readLen == EWOULDBLOCK){
          log.error() << "would block";
//              throw eeros::EEROSException("socket would block");
        }
        if (readLen < 0) {
          if(readLen != -1) log.info() << "error " << readLen;
        }
        if (readLen > 0) {
                for (int i = 0; i < nodes.size(); i++) {
                    if (readFrame.id == nodes[i]) {     // check if frame.id (node) is registered
                        mIt = nodePdoData[i].find(readFrame.function_code);
                        if (mIt != nodePdoData[i].end()) {
                            if(readFrame.function_code == CANOPEN_FC_PDO1_TX) {
//                              std::cout << "PDO1 rec" << std::endl;
                                if(readFrame.data_len > 1) {
                                    uint16_t tmpStatus = (readFrame.payload.data[0] & 0xFF);
                                    tmpStatus |= (uint16_t(readFrame.payload.data[1] & 0xFF) << 8);
                                    outStatus[i]->getSignal().setValue(tmpStatus & 0x26F);
                                    outStatus[i]->getSignal().setTimestamp(ts);
                                }   
                                if(readFrame.data_len > 5){
                                    int32_t tmpVel = (readFrame.payload.data[2] & 0x00FF);
                                    tmpVel |= ((readFrame.payload.data[3] & 0x00FF) << 8);
                                    tmpVel |= ((readFrame.payload.data[4] & 0x00FF) << 16);
                                    tmpVel |= ((readFrame.payload.data[5] & 0x00FF) << 24);
                                    outVel[i]->getSignal().setValue(static_cast<int32_t>(tmpVel) * outVelScale[i]);
                                    outVel[i]->getSignal().setTimestamp(ts);
                               }
                            } else if (readFrame.function_code == CANOPEN_FC_PDO2_TX) {
//                              std::cout << "PDO2 rec" << std::endl;
                                if (readFrame.data_len > 3) {
                                    int32_t tmpWarning = (readFrame.payload.data[0] & 0x00FF);
                                    tmpWarning |= ((readFrame.payload.data[1] & 0x00FF) << 8);
                                    tmpWarning |= ((readFrame.payload.data[2] & 0x00FF) << 16);
                                    tmpWarning |= ((readFrame.payload.data[3] & 0x00FF) << 24);
                                    outWarning[i]->getSignal().setValue(static_cast<int32_t>(tmpWarning));
                                     outWarning[i]->getSignal().setTimestamp(ts);
                               }
//                              if (readFrame.data_len > 7) {
//                                  uint32_t tmpWarning = (readFrame.payload.data[4] & 0x00FF);
//                                  tmpWarning |= ((readFrame.payload.data[5] & 0x00FF) << 8);
//                                  tmpWarning |= ((readFrame.payload.data[6] & 0x00FF) << 16);
//                                  tmpWarning |= ((readFrame.payload.data[7] & 0x00FF) << 24);
//                                  outWarning[i]->getSignal().setValue(tmpWarning) * outPosScale[i];
//                                     outWarning[i].getSignal().setTimestamp(ts);
//                              }
                            } else std::cout << "other PDO" << std::endl;
                        }
                    }
                }
            }
      }
    }
  }

  virtual Output<uint16_t>& getOutStatus(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outStatus[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }
  
  virtual Output<double>& getOutPos(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outPos[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }
  
  virtual Output<double>& getOutVel(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outVel[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }
  
  virtual Output<uint32_t>& getOutWarning(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outWarning[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }

  virtual void enable() {
    enabled = true;
  }
  
  virtual void disable() {
    enabled = false;
  }
  
  virtual void setPosScale(uint8_t node, double scale) {
    outPosScale[node] = scale;
  }

  virtual void setVelScale(uint8_t node, double scale) {
    outVelScale[node] = scale;
  }

		struct PdoUserData{
			uint8_t functionCode;
			uint16_t drvCtrl;
			int32_t userData;
		};	

 private:
  int socket;
  bool enabled = false;
		std::map<uint32_t, PdoUserData>::iterator mIt;
  Logger log;
  std::vector<Output<uint16_t>*> outStatus;
  std::vector<Output<double>*> outPos;
  std::vector<double> outPosScale;
  std::vector<Output<double>*> outVel;
  std::vector<double> outVelScale;
  std::vector<Output<uint32_t>*> outWarning;
  std::vector<uint8_t> nodes;
  std::vector<uint8_t> functionCodes;
		std::vector<std::map<uint32_t, PdoUserData>> nodePdoData;
	};
};
}

#endif // ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
