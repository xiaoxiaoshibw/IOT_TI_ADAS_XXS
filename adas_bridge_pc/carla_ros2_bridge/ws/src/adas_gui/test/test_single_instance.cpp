// single_instance.hpp 单测：真实 flock(2) 行为，不 mock——文件锁本身就是
// 系统调用契约，用临时文件实测比伪造语义更可信。
#include <gtest/gtest.h>

#include <QString>
#include <QTemporaryDir>

#include "single_instance.hpp"

namespace {

using adas::gui::SingleInstanceLock;

TEST(SingleInstanceLock, FirstLockerSucceeds) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  SingleInstanceLock lock(dir.filePath("test.lock"));
  EXPECT_TRUE(lock.tryLock());
  EXPECT_TRUE(lock.held());
}

TEST(SingleInstanceLock, SecondLockerOnSamePathFails) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = dir.filePath("test.lock");
  SingleInstanceLock first(path);
  SingleInstanceLock second(path);
  ASSERT_TRUE(first.tryLock());
  EXPECT_FALSE(second.tryLock());
  EXPECT_FALSE(second.held());
}

TEST(SingleInstanceLock, ReleasingFirstAllowsSecondToAcquire) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = dir.filePath("test.lock");
  SingleInstanceLock first(path);
  SingleInstanceLock second(path);
  ASSERT_TRUE(first.tryLock());
  ASSERT_FALSE(second.tryLock());
  first.unlock();
  EXPECT_TRUE(second.tryLock());
}

TEST(SingleInstanceLock, DestructorReleasesLock) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = dir.filePath("test.lock");
  {
    SingleInstanceLock first(path);
    ASSERT_TRUE(first.tryLock());
  }  // 析构，应该释放
  SingleInstanceLock second(path);
  EXPECT_TRUE(second.tryLock());
}

TEST(SingleInstanceLock, TryLockIsIdempotentForSameOwner) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  SingleInstanceLock lock(dir.filePath("test.lock"));
  EXPECT_TRUE(lock.tryLock());
  EXPECT_TRUE(lock.tryLock());  // 自己再叫一次不该失败
}

}  // namespace
