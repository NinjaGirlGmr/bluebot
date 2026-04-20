// Copyright (c) 2024 Bluebot
// SPDX-License-Identifier: Apache-2.0

#include "gtest/gtest.h"
#include "bt_test_fixture.hpp"

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(
    new bluebot_bt_plugins::test::ROS2Environment);
  return RUN_ALL_TESTS();
}
