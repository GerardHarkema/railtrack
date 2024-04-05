
#include "DCCPacketQueueList.h"
#include <stdexcept>

DCCPacketQueue::DCCPacketQueue(void)
{
  return;
}

bool DCCPacketQueue::setup()
{

  semaphore = xSemaphoreCreateMutex();
  if(!semaphore){
    Serial.println("Error creating Semaphore"); 
    return false;   
  }
  return true;
}

bool DCCPacketQueue::insertPacket(DCCPacket &packet)
{
  bool result = true;
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
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
  xSemaphoreGive(semaphore);
  return result;
#if 0
//  Serial.println("Queue is full!");
  return false;
#endif
}

void DCCPacketQueue::printQueue(void)
{
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
  };  
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
  xSemaphoreGive(semaphore);
}

bool DCCPacketQueue::readPacket(DCCPacket &packet)
{
  bool result = false;
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };  
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
  xSemaphoreGive(semaphore);
  return result;
}

bool DCCPacketQueue::forget(uint16_t address, uint8_t address_kind)
{

  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool found = false;
  std::list<DCCPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    if(queue_packet->getAddress() == address && queue_packet->getKind() == address_kind){
      queue.erase(queue_packet);
      queue_full = false;
      found = true;
      break;
    }
  }
  xSemaphoreGive(semaphore);
  return found;
}

void DCCPacketQueue::clear(void)
{
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
  };
  queue.clear();
  xSemaphoreGive(semaphore);
}

