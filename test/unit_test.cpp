#include <gtest/gtest.h>

bool dummy_func() {
  return false;
}

TEST(dummy_test, dummy_func) {
  EXPECT_EQ(true, dummy_func());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}