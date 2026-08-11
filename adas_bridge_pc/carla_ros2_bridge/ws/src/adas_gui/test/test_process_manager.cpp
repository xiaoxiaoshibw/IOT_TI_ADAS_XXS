#include <gtest/gtest.h>

#include <QTcpServer>

#include "process_manager.hpp"

namespace adas::gui {

TEST(ProcessManager, HilIsRejectedWithoutStartingHardware) {
  ProcessManager manager;
  ProcState reported = ProcState::Stopped;
  QObject::connect(&manager, &ProcessManager::bridgeChanged,
                   [&reported](ProcState state, const QString&) { reported = state; });

  LaunchConfig config;
  config.backend = QStringLiteral("hil");
  manager.startAll(config);

  EXPECT_EQ(reported, ProcState::Failed);
  EXPECT_EQ(manager.bridgeState(), ProcState::Stopped);
  EXPECT_FALSE(manager.hasManagedProcesses());
}

TEST(ProcessManager, ExternalBridgeIsNotStoppedByGui) {
  ProcessManager manager;
  manager.setExternalBridgeDetected(true);

  EXPECT_FALSE(manager.hasManagedProcesses());
  EXPECT_TRUE(manager.externalBridgeDetected());

  manager.stopBridge();

  EXPECT_EQ(manager.bridgeState(), ProcState::Running);
  EXPECT_FALSE(manager.hasManagedProcesses());
}

TEST(ProcessManager, MilRejectsWrongDdsDomainBeforeSpawning) {
  const QByteArray previous = qgetenv("ROS_DOMAIN_ID");
  qputenv("ROS_DOMAIN_ID", "43");
  ProcessManager manager;
  ProcState reported = ProcState::Stopped;
  QObject::connect(&manager, &ProcessManager::bridgeChanged,
                   [&reported](ProcState state, const QString&) { reported = state; });
  LaunchConfig config;
  config.backend = QStringLiteral("mil");
  manager.startAll(config);

  EXPECT_EQ(reported, ProcState::Failed);
  EXPECT_EQ(manager.bridgeState(), ProcState::Stopped);
  EXPECT_FALSE(manager.hasManagedProcesses());
  if (previous.isNull()) qunsetenv("ROS_DOMAIN_ID");
  else qputenv("ROS_DOMAIN_ID", previous);
}

TEST(ProcessManager, ExternalBridgeDisappearanceRestoresStoppedState) {
  ProcessManager manager;
  manager.setExternalBridgeDetected(true);
  ASSERT_EQ(manager.bridgeState(), ProcState::Running);

  manager.setExternalBridgeDetected(false);

  EXPECT_EQ(manager.bridgeState(), ProcState::Stopped);
}

TEST(ProcessManager, RejectedFlashAlwaysCompletesForUiRecovery) {
  ProcessManager manager;
  int completions = 0;
  bool success = true;
  QObject::connect(&manager, &ProcessManager::flashFinished,
                   [&completions, &success](bool result, const QString&) {
                     ++completions;
                     success = result;
                   });

  manager.flashMcuFirmware(QString());

  EXPECT_EQ(completions, 1);
  EXPECT_FALSE(success);
}

TEST(ProcessManager, ExistingMilRuntimeForcesObserverMode) {
  const QByteArray previous = qgetenv("ADAS_LOCAL_THREE_MACHINE");
  qputenv("ADAS_LOCAL_THREE_MACHINE", "1");
  {
    ProcessManager manager;
    EXPECT_TRUE(manager.localThreeMachineObserver());
    LaunchConfig config;
    config.backend = QStringLiteral("mil");
    manager.startAll(config);
    EXPECT_EQ(manager.bridgeState(), ProcState::Running);
    EXPECT_FALSE(manager.hilManagerActive());
    manager.stopAll();
    EXPECT_FALSE(manager.hilManagerActive());
  }
  if (previous.isNull()) qunsetenv("ADAS_LOCAL_THREE_MACHINE");
  else qputenv("ADAS_LOCAL_THREE_MACHINE", previous);
}

}  // namespace adas::gui
