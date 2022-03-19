/* Krishna Mani
 * Pipe.h
 */
#ifndef GENERICPIPE_H
#define GENERICPIPE_H

#include <mutex>
#include <deque>
#include <string>
#include <vector>

class Message {
  public:
  Message();
  Message(std::string, std::vector<float>);
  Message(std::string, int);
  std::string str;
  std::vector<float> vals;
};

class GenericPipe {
  std::mutex _mutex;
  std::deque<Message*> _deque;

  public:
  void pushQueue(Message*);
  Message* popQueue();
};

#endif
