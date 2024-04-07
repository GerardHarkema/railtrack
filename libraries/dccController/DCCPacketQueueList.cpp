
#include "DCCPacketQueueList.h"
#include <stdexcept>

DCCPacketQueue::DCCPacketQueue(void)
{
  return;
}

bool DCCPacketQueue::setup()
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

bool DCCPacketQueue::insertPacket(DCCPacket &packet)
{

  bool result = true;
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
#endif
  std::list<DCCPacket>::iterator queue_packet;
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

void DCCPacketQueue::printQueue(void)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
  };  
#endif
  int i = 0;
  std::list<DCCPacket>::iterator queue_packet;
#if 1
  if(queue.size()){
    for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
      Serial.printf("%i: Address = 0x%04x, Kind = %i, Repeatcount = %i", i,
        queue_packet->getAddress(), 
        queue_packet->getKind(), 
        queue_packet->getRepeatCount());
      u_int8_t data_size = queue_packet->getSize();
      Serial.printf(", Size = %i", data_size);
      if(data_size){
        Serial.printf(", Data =");
        for(int j = 0; j < data_size; j++)
          Serial.printf(" 0x%02x", queue_packet->getData(j));
      }
      Serial.printf("\n");
      i++;
    }
  }
  else{
    Serial.printf("Queue Empty\n");    
  }
#endif
#ifdef THREAD_SAFE_QUEUE
  xSemaphoreGive(semaphore);
#endif
}

bool DCCPacketQueue::readPacket(DCCPacket &packet)
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

bool DCCPacketQueue::forget(uint16_t address, uint8_t address_kind)
{
#ifdef THREAD_SAFE_QUEUE
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
#endif
  bool found = false;
  std::list<DCCPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    Serial.printf("Packet address = %04x, kind = %i\n", queue_packet->getAddress(), queue_packet->getKind());
    Serial.printf("Request address = %04x, kind = %i\n", address, address_kind);
    if((queue_packet->getAddress() == address) && (queue_packet->getKind() == address_kind)){
      Serial.printf("Erase packet\n");
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

void DCCPacketQueue::clear(void)
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

