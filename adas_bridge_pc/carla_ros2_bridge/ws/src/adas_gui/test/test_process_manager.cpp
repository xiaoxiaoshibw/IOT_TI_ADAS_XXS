#include <gtest/gtest.h>

#include <QTcpServer>

#include "process_manager.hpp"

namespace adas::gui {

TEST(ProcessManager, RosGraphBridgeIsReused) {
  ProcessManager manager;
  manager.setBridgeProbe([]() { return true; });

  LaunchConfig config;
  manager.startBridge(config);

  EXPECT_EQ(manager.bridgeState(), ProcState::Running);
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

TEST(ProcessManager, ExternalCarlaIsNotStoppedByGui) {
  QTcpServer server;
  ASSERT_TRUE(server.listen(QHostAddress::LocalHost, 0));

  ProcessManager manager;
  LaunchConfig config;
  config.carla_port = server.serverPort();
  manager.startCarla(config);

  EXPECT_TRUE(manager.externalCarlaDetected());
  EXPECT_FALSE(manager.hasManagedProcesses());

  manager.stopAll();

  EXPECT_TRUE(server.isListening());
  EXPECT_TRUE(manager.externalCarlaDetected());
  EXPECT_FALSE(manager.hasManagedProcesses());
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

}  // namespace adas::gui
