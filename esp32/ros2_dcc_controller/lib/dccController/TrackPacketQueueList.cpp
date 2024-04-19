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
    Serial.println("Error creating Semaphore"); 
    return false;   
  }
#endif
  return true;
}

bool TrackPacketQueue::insertPacket(TrackPacket &packet)
{

  bool result = true;
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
#endif
  std::list<TrackPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    if(queue_packet->getAddress() == packet.getAddress() && queue_packet->getKind() == packet.getKind() ){
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
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
  return result;
#if 0
//  Serial.println("Queue is full!");
  return false;
#endif
}

void TrackPacketQueue::printQueue(void)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
  };  
#endif
  int i = 0;
  std::list<TrackPacket>::iterator queue_packet;
#if 1
  if(queue.size()){
    for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
      DEBUG_PRINT("%i: Address = 0x%04x, Kind = %i, Repeatcount = %i", i,
        queue_packet->getAddress(), 
        queue_packet->getKind(), 
        queue_packet->getRepeatCount());
      u_int8_t data_size = queue_packet->getSize();
      DEBUG_PRINT(", Size = %i", data_size);
      if(data_size){
        DEBUG_PRINT(", Data =");
        for(int j = 0; j < data_size; j++)
          DEBUG_PRINT(" 0x%02x", queue_packet->getData(j));
      }
      DEBUG_PRINT("\n");
      i++;
    }
  }
  else{
    DEBUG_PRINT("Queue Empty\n");    
  }
#endif
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
}

bool TrackPacketQueue::readPacket(TrackPacket &packet)
{
  bool result = false;
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
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

bool TrackPacketQueue::forget(uint16_t address, uint8_t address_kind)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
#endif
  bool found = false;
  std::list<TrackPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    DEBUG_PRINT("Packet address = %04x, kind = %i\n", queue_packet->getAddress(), queue_packet->getKind());
    DEBUG_PRINT("Request address = %04x, kind = %i\n", address, address_kind);
    if((queue_packet->getAddress() == address) && (queue_packet->getKind() == address_kind)){
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
    Serial.println("Error taking Semaphore");
  };
#endif
  queue.clear();
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
}

