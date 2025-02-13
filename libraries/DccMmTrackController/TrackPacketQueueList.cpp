#include "TrackPacketQueueList.h"
#include <stdexcept>

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

TrackPacketQueue::TrackPacketQueue(void)
{
  return;
}

bool TrackPacketQueue::setup()
{

#ifdef THREAD_SAFE_QUEUE
  semaphore = xSemaphoreCreateMutex();
  if(!semaphore){
    DEBUG_PRINT("Error creating Semaphore\n"); 
    return false;   
  }
#endif
  return true;
}

bool TrackPacketQueue::insertPacket(TrackPacket &packet)
{

  bool found = false;
  bool result = true;
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    DEBUG_PRINT("Error taking Semaphore\n");
    return false;
  };
#endif
  std::list<TrackPacket>::iterator queue_packet;
  switch(packet.getPacketProtocol()){
    case TRACK_PROTOCOL_DCC:
      for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
        if(queue_packet->dccGetAddress() == packet.dccGetAddress() && 
            queue_packet->dccGetKind() == packet.dccGetKind() && 
            queue_packet->getPacketProtocol() == TRACK_PROTOCOL_DCC){
          queue.erase(queue_packet);
          break;
        }
      }
      try {
        if(packet.isHighPriority())
          queue.push_front(packet);
        else
          queue.push_back(packet);
      }
      catch (const std::exception& e){
        queue_full = true;
        result = false;
      }
      break;
    case TRACK_PROTOCOL_MM:
      if(packet.mmGetKind() == MM1_LOC_CHANGE_DIR_TELEGRAM){
        try {
            if(packet.isHighPriority())
              queue.push_front(packet);
            else
              queue.push_back(packet);
          }
          catch (const std::exception& e){
            queue_full = true;
            result = false;
          }
      }

      for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
        if(queue_packet->mmGetAddress() == packet.mmGetAddress() && 
            queue_packet->getPacketProtocol() == TRACK_PROTOCOL_MM){
          // Update speed of all matching telegrams
          switch(queue_packet->mmGetKind())
          {
            case MM1_LOC_SPEED_TELEGRAM:
              found = true;
#if 0
              if(packet.mmGetKind() == MM1_LOC_SPEED_TELEGRAM){
                 Serial.printf("found\n");

                found = true;
              }
#endif
              switch(packet.mmGetKind()){
                case MM1_LOC_SPEED_TELEGRAM:
                  queue_packet->mmSetSpeed(packet.mmGetSpeed());
                  Serial.printf("MM1_LOC_SPEED_TELEGRAM\n");
                  break;
                case MM_LOC_AUXILIARY_TELEGRAM:
                  queue_packet->mmSetAuxiliary(packet.mmGetAuxiliary());
                  break;
              }
              break;
            case MM1_LOC_F_TELEGRAM:
              found = true;
              for(int i = 0; i < 4; i++)
                queue_packet->mmSetFunction(i, packet.mmGetFunction(i));
              break;
            case MM2_LOC_SPEED_TELEGRAM:
            case MM2_LOC_F1_TELEGRAM:
            case MM2_LOC_F2_TELEGRAM:
            case MM2_LOC_F3_TELEGRAM:
            case MM2_LOC_F4_TELEGRAM:
              found = true;
              switch(packet.mmGetKind()){
                case MM2_LOC_SPEED_TELEGRAM:
                  queue_packet->mmSetSpeed(packet.mmGetSpeed());
                  break;
                case MM2_LOC_F1_TELEGRAM:
                  queue_packet->mmSetFunction(0, packet.mmGetFunction());
                  break;
                case MM2_LOC_F2_TELEGRAM:
                  queue_packet->mmSetFunction(1, packet.mmGetFunction());
                  break;
                case MM2_LOC_F3_TELEGRAM:
                  queue_packet->mmSetFunction(2, packet.mmGetFunction());
                  break;
                case MM2_LOC_F4_TELEGRAM:
                  queue_packet->mmSetFunction(3, packet.mmGetFunction());
                  break;
                case MM_LOC_AUXILIARY_TELEGRAM:
                  queue_packet->mmSetAuxiliary(packet.mmGetAuxiliary());
                  break;
              }
              break;
            case MM_SOLENOID_TELEGRAM:
              
              // telegram always added
              break;
          }
          //queue.erase(queue_packet);
          break;
        }
      }
      if(!found){
        //DEBUG_PRINT("MM Telegram inserted\n");
        if(packet.mmGetKind() != MM_LOC_AUXILIARY_TELEGRAM){ // skip this fake telegram
                         Serial.printf("insert\n");

          try {
            if(packet.isHighPriority())
              queue.push_front(packet);
            else
              queue.push_back(packet);
          }
          catch (const std::exception& e){
            queue_full = true;
            result = false;
          }
        }
      }
      break;
  }
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
  return result;
#if 0
//  DEBUG_PRINT("Queue is full!\n");
  return false;
#endif
}

void TrackPacketQueue::printQueue(void)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    DEBUG_PRINT("Error taking Semaphore\n");
  };  
#endif
  int i = 0;
  u_int8_t dcc_bitstream_size;
  std::list<TrackPacket>::iterator queue_packet;

  if(queue.size()){
    for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
      switch(queue_packet->getPacketProtocol()){
        case TRACK_PROTOCOL_DCC:
          DEBUG_PRINT("Queue-> %i (dcc): Address = 0x%04x, Kind = %i, Repeatcount = %i", i,
            queue_packet->dccGetAddress(), 
            queue_packet->dccGetKind(), 
            queue_packet->getRepeatCount());
            dcc_bitstream_size = queue_packet->dccGetSize();
            DEBUG_PRINT(", Size = %i", dcc_bitstream_size);
            if(dcc_bitstream_size){
              DEBUG_PRINT(", Data = ");
              for(int j = 0; j < dcc_bitstream_size; j++)
                DEBUG_PRINT(" 0x%02x", queue_packet->dccGetData(j));
            }
            DEBUG_PRINT("\n");
          break;
        case TRACK_PROTOCOL_MM:
          DEBUG_PRINT("Queue-> %i (mm): Address = 0x%04x, Kind = %i, Repeatcount = %i", i,
            queue_packet->mmGetAddress(), 
            queue_packet->mmGetKind(), 
            queue_packet->getRepeatCount());
            DEBUG_PRINT(", Size = 76(fixed)");
            DEBUG_PRINT("\n");
          break;
      }
      i++;
    }
  }
  else{
    DEBUG_PRINT("Queue->Queue Empty\n");    
  }

#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
}

bool TrackPacketQueue::readPacket(TrackPacket &packet)
{
  bool result = false;
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    DEBUG_PRINT("Error taking Semaphore\n");
    return false;
  };  
#endif
  if(!isEmpty()){
    packet = queue.front();
    queue.pop_front();
    if(packet.getPacketProtocol() == TRACK_PROTOCOL_MM){
       packet.mmSetNextKind();
    }
    int8_t repeat_count =  packet.getRepeatCount();
    if(repeat_count == REPEAT_COUNT_CONTINOUS) queue.push_back(packet);
    else{
      if(repeat_count > 0) 
      {
        packet.setRepeatCount(repeat_count-1);
        if(packet.isHighPriority()) queue.push_front(packet);
        else queue.push_back(packet);
      }
    }
  }
#if 0
    }
   queue_full = false;
    result = true;
  }
#endif
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
  return result;
}

bool TrackPacketQueue::forget(uint16_t address, uint8_t dcc_address_kind)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    DEBUG_PRINT("Error taking Semaphore\n");
    return false;
  };
#endif
  bool found = false;
  std::list<TrackPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    DEBUG_PRINT("Packet address = %04x, kind = %i\n", queue_packet->dccGetAddress(), queue_packet->dccGetKind());
    DEBUG_PRINT("Request address = %04x, kind = %i\n", address, dcc_address_kind);
    if((queue_packet->dccGetAddress() == address) && (queue_packet->dccGetKind() == dcc_address_kind)){
      DEBUG_PRINT("Erase packet\n");
      queue.erase(queue_packet);
      queue_full = false;
      found = true;
      break;
    }
  }
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
  return found;
}

void TrackPacketQueue::clear(void)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    DEBUG_PRINT("Error taking Semaphore\n");
  };
#endif
  queue.clear();
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
}

