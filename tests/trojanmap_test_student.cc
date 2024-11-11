#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, Test1) {
  EXPECT_EQ(true, true);
}

// Phase 1




// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  
  // Test Tommy Trojan
  auto position = m.GetPosition("Tommy Trojan");
  std::pair<double, double> gt1(34.0205678, -118.2854346); // groundtruth for "Tommy Trojan"
  EXPECT_EQ(position, gt1);
  // Test Cosmo Plaza
  position = m.GetPosition("Cosmo Plaza");
  std::pair<double, double> gt2(34.0396911, -118.2547056); // groundtruth for "Cosmo Plaza"
  EXPECT_EQ(position, gt2);
  // Test The Mirage
  position = m.GetPosition("The Mirage");
  std::pair<double, double> gt3(34.0275051,-118.2829909); // groundtruth for "The Mirage"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("Minecraft Library");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}


TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Ro");
  std::unordered_set<std::string> gt = {"Rock & Reillys","Ross","Rossoblu",
  "Roger Williams Baptist Church"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("ro");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("rO"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("RO"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}
