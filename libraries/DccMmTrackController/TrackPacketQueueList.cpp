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
      for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
        if(queue_packet->mmGetAddress() == packet.mmGetAddress() && 
            queue_packet->mmGetKind() == packet.mmGetKind() &&
            queue_packet->getPacketProtocol() == TRACK_PROTOCOL_MM){
          // Update speed of all matching telegrams
          switch(queue_packet->mmGetKind())
          {
            case MM2_LOC_SPEED_TELEGRAM:
              if(packet.mmGetKind() == MM2_LOC_SPEED_TELEGRAM)
                queue_packet->mmSetSpeed(packet.mmGetSpeed());
              if(queue_packet->mmGetKind() == packet.mmGetKind()) found = true;
              break;
            case MM2_LOC_F1_TELEGRAM:
              if(packet.mmGetKind() == MM2_LOC_F1_TELEGRAM)
                queue_packet->mmSetFunction(packet.mmGetFunction());
              else
                queue_packet->mmSetSpeed(packet.mmGetSpeed());
              if(queue_packet->mmGetKind() == packet.mmGetKind()) found = true;
              break;
            case MM2_LOC_F2_TELEGRAM:
              if(packet.mmGetKind() == MM2_LOC_F3_TELEGRAM)
                queue_packet->mmSetFunction(packet.mmGetFunction());
              else
                queue_packet->mmSetSpeed(packet.mmGetSpeed());
              if(queue_packet->mmGetKind() == packet.mmGetKind()) found = true;
              break;
            case MM2_LOC_F3_TELEGRAM:
              if(packet.mmGetKind() == MM2_LOC_F3_TELEGRAM)
                queue_packet->mmSetFunction(packet.mmGetFunction());
              else
                queue_packet->mmSetSpeed(packet.mmGetSpeed());
              if(queue_packet->mmGetKind() == packet.mmGetKind()) found = true;
              break;
            case MM2_LOC_F4_TELEGRAM:
              if(packet.mmGetKind() == MM2_LOC_F4_TELEGRAM)
                queue_packet->mmSetFunction(packet.mmGetFunction());
              else
                queue_packet->mmSetSpeed(packet.mmGetSpeed());
              if(queue_packet->mmGetKind() == packet.mmGetKind()) found = true;
              break;
            case MM2_MAGNET_TELEGRAM:
              // telegram always added
              break;
          }
          //queue.erase(queue_packet);
          //break;
          if(!found){
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
          Serial.printf("Queue-> %i (dcc): Address = 0x%04x, Kind = %i, Repeatcount = %i", i,
            queue_packet->dccGetAddress(), 
            queue_packet->dccGetKind(), 
            queue_packet->getRepeatCount());
            dcc_bitstream_size = queue_packet->dccGetSize();
            Serial.printf(", Size = %i", dcc_bitstream_size);
            if(dcc_bitstream_size){
              DEBUG_PRINT(", Data =\n");
              for(int j = 0; j < dcc_bitstream_size; j++)
                Serial.printf(" 0x%02x", queue_packet->dccGetData(j));
            }
            Serial.printf("\n");
          break;
        case TRACK_PROTOCOL_MM:
          Serial.printf("Queue-> %i (mm): Address = 0x%04x, Kind = %i, Repeatcount = %i", i,
            queue_packet->mmGetAddress(), 
            queue_packet->mmGetKind(), 
            queue_packet->getRepeatCount());
            Serial.printf(", Size = 76(fixed)");
            Serial.printf("\n");
          break;
      }
      i++;
    }
  }
  else{
    Serial.printf("Queue->Queue Empty\n");    
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

