#ifndef DRIVE_FLUSHTHREAD_H_
#define DRIVE_FLUSHTHREAD_H_

#include <pthread.h>
#include <semaphore.h>

#include <deque>

// asynchronous flush to sdcard
struct FlushEntry {
  int fd_;
  uint8_t *buf_;
  size_t len_;
  size_t unsynced_;

  FlushEntry() { buf_ = NULL; }
  FlushEntry(int fd, uint8_t *buf, size_t len):
    fd_(fd), buf_(buf), len_(len) { unsynced_ = 0; }

  void flush() {
    if (len_ == -1) {
      fprintf(stderr, "FlushThread: closing fd %d\n", fd_);
      close(fd_);
    }
    if (buf_ != NULL) {
      if (write(fd_, buf_, len_) != len_) {
        perror("FlushThread write");
      }
      delete[] buf_;
      buf_ = NULL;
      unsynced_ += len_;
      // sync every 1MB
      // way too expensive! wtf!
      if (unsynced_ > 1048576) {
        unsynced_ = 0;
        fsync(fd_);
      }
    }
  }
};

class FlushThread {
 public:
  FlushThread() {
    pthread_mutex_init(&mutex_, NULL);
    sem_init(&sem_, 0, 0);
  }

  ~FlushThread() {
    // terminate the thread?
  }

  bool Init() {
    if (pthread_create(&thread_, NULL, thread_entry, this) != 0) {
      perror("FlushThread: pthread_create");
      return false;
    }
    return true;
  }

  void AddEntry(int fd, uint8_t *buf, size_t len) {
    static int count = 0;
    pthread_mutex_lock(&mutex_);
    flush_queue_.push_back(FlushEntry(fd, buf, len));
    size_t siz = flush_queue_.size();
    pthread_mutex_unlock(&mutex_);
    sem_post(&sem_);
    count++;
    if (count >= 15) {
      if (siz > 2) {
        fprintf(stderr, "[FlushThread %d]\r", siz);
        fflush(stderr);
      }
      count = 0;
    }
#if 0
    int semval;
    sem_getvalue(&sem_, &semval);
    fprintf(stderr, "Flusher: qsize %d sem %d\n", flush_queue_.size(), semval);
#endif
  }

 private:
  static void* thread_entry(void* arg) {
    FlushThread *self = reinterpret_cast<FlushThread*>(arg);

    fprintf(stderr, "FlushThread: started\n");

    for (;;) {
      sem_wait(&self->sem_);
      pthread_mutex_lock(&self->mutex_);
      if (!self->flush_queue_.empty()) {
        FlushEntry e = self->flush_queue_.front();
        self->flush_queue_.pop_front();
        pthread_mutex_unlock(&self->mutex_);
        e.flush();
      } else {
        pthread_mutex_unlock(&self->mutex_);
      }
    }
  }

  std::deque<FlushEntry> flush_queue_;
  pthread_mutex_t mutex_;
  pthread_t thread_;
  sem_t sem_;
};

#endif  // DRIVE_FLUSHTHREAD_H_
