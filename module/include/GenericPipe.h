// /* Krishna Mani
//  * Pipe.h
//  */
// #ifndef GENERICPIPE_H
// #define GENERICPIPE_H

// #include <mutex>
// #include <deque>
// #include <string>

// class Message {
//   public:
//   Message();
//   Message(std::string, float);
//   std::string str;
//   float val;
// };

// class GenericPipe {
//   std::mutex _mutex;
//   std::deque<Message*> _deque;

//   public:
//   void pushQueue(Message*);
//   Message* popQueue();
// };

// #endif
