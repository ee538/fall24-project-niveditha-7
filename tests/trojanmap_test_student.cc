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

//Phase2
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  
  // Test from Trojan Grounds (Starbucks) to Ralphs
  auto path = m.CalculateShortestPath_Dijkstra("Trojan Grounds (Starbucks)", "Ralphs");
  std::vector<std::string> gt{
      "614990288","1673644992","6819179775","3233762100","1771091127","6819179774",
      "6787896179","5580902573","6814770343","3431300454","6814770351","3432332948",
      "4536989636","3433701978","6818390136","6813379491","3443310465","4536989640",
      "4536989637","6813379432","3402887075","6813379464","6813379465","6813379466",
      "6813379440","5690152766","4015423966","7434941012","6818390143","63068610",
      "6818390140","6813379476","21306059","544672028","6813379482","6813405206",
      "123318572","5565967545","7811699597","6817230310","3642819026","6817230316",
      "2613117861","6818390170","6818390171","6818390172","6807374562","2613117885",
      "6818390165","2613117882","6818390178","2613117902","6813416153","6813416154",
      "6813416145","7232024780","6818427916","6818427917","6818427898","6818427892",
      "6818427918","6818427919","6818427920","4380040148","4380040152","4380040153",
      "4380040154","2578244375"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath_Dijkstra( "Ralphs", "Trojan Grounds (Starbucks)");
  std::reverse(gt.begin(),gt.end()); // Reverse the path


  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Bellman_Ford("Chipotle", "DASH");
  std::vector<std::string> gt{
      "732641023","9446678100","6820935908","216155217","6813411589","1837212103",
      "1837212101","6820935911","932416162","4060015482","4020099365","6820972450",
      "1732243549","6820972451","1836105028","9118747759","4060015481","4020099358",
      "6814990127","932378141","932378142","1869431093","9118747761","1869431099",
      "1870800157","1870800154","1870800155","1870800153","932378197","1870800152",
      "932378196","932378195","932378167","1472141010","123166173","5707881721","213431767",
      "213431754","1836106815","1836119958","1836106817","1836106818","932387853","213431728",
      "6693451972","1836105952","1836106812","213431722","213431715","1836106811","123166178",
      "1836106814","1862312619","123166179","1855173116","1630951165","1855143762","1855143765",
      "1855143757","1630951168","6814481791","1630951209","6814481787","1630940683","6814481788",
      "6814481789","4020099328","7591225398","1832254598","6814990096","6815190469","5618016520",
      "7872111891","5618016821","5618016824","5618016825","6814452683","5618016826","5618016828",
      "5618016829","5618016830","7257246599","5618016833","123152329","7257246593","4020099320",
      "5618016838","123152331","5618016841","4020099318","6512331875","6512331876"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("DASH", "Chipotle");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, CalculateShortestPath_SameNode) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Dijkstra("Target", "Target");
  std::vector<std::string> gt{};
  EXPECT_EQ(path, gt);
}

//Phase2
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;
  
  // Test from Trojan Grounds (Starbucks) to Ralphs
  auto path = m.CalculateShortestPath_Dijkstra("Dulce", "The Barber Shop");
  std::vector<std::string> gt{
      "5567718696","6814820015","6814820010","6814820018","6814769290","9591449485","6396649383",
      "6814769289","6813379584","6813379479","3398578901","6813379425","4399698015","3398578900",
      "4399698005","6813379519","6813379505","6813379398","3398578898","6813565290","3398574892",
      "3398578893","2613117879","6813379406","6807905595","6787803635","2613117867","6813565334",
      "4835551105","2613117915","2613117890","3403034590","2613117862","4835551093","3403034586",
      "2613117900","4835551084","7863689395","7863689394","3403035500","3403035499","7693467754",
      "5680945525","5556118325","6816193705","3403035498","6813565323","3398621871","6813565325",
      "5680944619","5680944620","6816959869","6816959863","122454252","1832234144","6816193810",
      "1832234142","4258846991","1832234141","8501336167","6817111153","4011837239","123161907",
      "6787830192","6787830199","123241958","123241955","7362236521","123241952","7362236512",
      "4012864457","7863404947","21098545","7225140904","8501336165","4019981462","7225140900",
      "7227363544","4019974803","6816950645","123241947","123241944","122420459","4020001608",
      "7642589440","60957897","4020023719","123241939","4020023720","3846520096","5567738306"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath_Dijkstra("The Barber Shop", "Dulce");
  std::reverse(gt.begin(),gt.end()); // Reverse the path


  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// PHASE II
// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("USC", "UCLA"), 3);
  EXPECT_EQ(m.CalculateEditDistance("kitten", "sitten"), 1);
  EXPECT_EQ(m.CalculateEditDistance("lol", "LOL"), 0);
}

// Test get all categories
TEST(TrojanMapTest, GetAllCategories) {
  TrojanMap m;
  
  auto output = m.GetAllCategories();
  std::set<std::string> expected = {
    "artwork", "attraction", "bakery", "bank", "bar", "beauty", "beverages", "bicycle", "bicycle_rental",
    "bus_station", "cafe", "car", "car_repair", "car_wash", "charging_station", "childcare", "clinic",
    "clothes", "confectionery", "convenience", "copyshop", "dentist", "department_store", "driving_school",
    "fabric", "fast_food", "food_court", "fountain", "fuel", "gallery", "hairdresser", "hospital", "hotel",
    "library", "marketplace", "mobile_phone", "museum", "music", "optician", "parcel_locker", "parking",
    "parking_entrance", "pharmacy", "place_of_worship", "police", "post_box", "post_office", "restaurant",
    "school", "shoe_repair", "shoes", "skate", "social_facility", "supermarket", "theatre", "tobacco",
    "yoga", "yes"
  };
  std::set<std::string> output_set(output.begin(), output.end());
  EXPECT_EQ(output_set, expected);
}

//Test GetAllLocationsFRom Category
TEST(TrojanMapTest, GetAllLocationsFromCategory) {
  TrojanMap m;
  
  auto output = m.GetAllLocationsFromCategory("museum");
  std::set<std::string> expected = {"4399693647"};
  std::set<std::string> output_set(output.begin(), output.end());
  EXPECT_EQ(output_set, expected);
}

TEST(TrojanMapTest, GetAllLocationsFromCategory1) {
  TrojanMap m;
  
  auto output = m.GetAllLocationsFromCategory("music");
  std::set<std::string> expected = {"5695174693", "5695183055"};
  std::set<std::string> output_set(output.begin(), output.end());
  EXPECT_EQ(output_set, expected);
}

//Test GetLocation Regex
TEST(TrojanMapTest, GetLocationRegex) {
  TrojanMap m;
  std::set<std::string> expected_set;
  auto actual = m.GetLocationRegex(std::regex("dummy"));
  std::set<std::string> actual_set(actual.begin(), actual.end());
  EXPECT_EQ(actual_set, expected_set);
}

TEST(TrojanMapTest, GetLocationRegex1) {
  TrojanMap m;
  std::set<std::string> expected_set = { "4927493958" };
  auto actual = m.GetLocationRegex(std::regex("Five Guys"));
  std::set<std::string> actual_set(actual.begin(), actual.end());
  EXPECT_EQ(expected_set, actual_set);
}



// Test topological sort function for DeliveringTrojan
TEST(TrojanMapTest, TopologicalSortTest) {
  TrojanMap m;

  // Test case 1
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {
      {"Ralphs", "KFC"},
      {"Ralphs", "Chick-fil-A"},
      {"KFC", "Chick-fil-A"}
  };
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt = {"Ralphs", "KFC", "Chick-fil-A"};
  EXPECT_EQ(result, gt);

  // Test case 2
  location_names = {"Starbucks", "Exposition Park", "Target", "Food 4 Less"};
  dependencies = {
      {"Starbucks", "Exposition Park"},
      {"Exposition Park", "Target"},
      {"Target", "Food 4 Less"}
  };
  result = m.DeliveringTrojan(location_names, dependencies);
  gt = {"Starbucks", "Exposition Park", "Target", "Food 4 Less"};
  EXPECT_EQ(result, gt);

  // Test case 3
  location_names = {"Department of Motor Vehicles", "Chick-fil-A", "Ralphs"};
  dependencies = {
      {"Department of Motor Vehicles", "Chick-fil-A"},
      {"Ralphs", "Chick-fil-A"}
  };
  result = m.DeliveringTrojan(location_names, dependencies);
  gt = {"Ralphs", "Department of Motor Vehicles", "Chick-fil-A"};
  EXPECT_EQ(result, gt);
}



// Test cycle detection function
TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);
}

// Test cycle detection function
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.299, 34.032, 34.011}; //not a square test
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, false);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.275, 34.032, 34.011};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, true);
}
//Test TopologicalSort
// Test cycle detection function with 5 nodes
TEST(TrojanMapTest, TopologicalSort5) {
  TrojanMap m;

  // Define locations and dependencies
  std::vector<std::string> location_names = {"CAVA", "Tommy Trojan", "USC Fisher Museum of Art", "Haagen Dazs Direct", "Hollywood Wraps"};
  std::vector<std::vector<std::string>> dependencies = {
      {"CAVA", "USC Fisher Museum of Art"},  // Must visit CAVA before USC Fisher Museum of Art
      {"CAVA", "Haagen Dazs Direct"},  // Must visit CAVA before Haagen Dazs Direct
      {"Tommy Trojan", "Hollywood Wraps"},  // Must visit Tommy Trojan before Hollywood Wraps
      {"USC Fisher Museum of Art", "Hollywood Wraps"},        // Must visit USC Fisher Museum of Art before Hollywood Wraps
      {"Haagen Dazs Direct", "Hollywood Wraps"}         // Must visit Haagen Dazs Direct before Hollywood Wraps
  };
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt = {"Tommy Trojan", "CAVA", "USC Fisher Museum of Art", "Haagen Dazs Direct", "Hollywood Wraps"};
  EXPECT_EQ(result, gt);
}

// Test cycle detection function with 8 nodes
TEST(TrojanMapTest, TopologicalSortWithEightNodes) {
  TrojanMap m;
  std::vector<std::string> location_names = {
      "CAVA", "Tommy Trojan", "USC Fisher Museum of Art", 
      "Haagen Dazs Direct", "Hollywood Wraps", "Holy Green", "Workshop Salon & Boutique", "Chase"
  };
  std::vector<std::vector<std::string>> dependencies = {
      {"CAVA", "Haagen Dazs Direct"},  // Must visit CAVA before Haagen Dazs Direct
      {"Tommy Trojan", "Hollywood Wraps"},  // Must visit Tommy Trojan before Hollywood Wraps
      {"USC Fisher Museum of Art", "Holy Green"},  // Must visit USC Fisher Museum of Art before Home3
      {"Haagen Dazs Direct", "Chase"},         // Must visit Haagen Dazs Direct before Chase
      {"Hollywood Wraps", "Workshop Salon & Boutique"},        // Must visit Hollywood Wraps before Workshop Salon & Boutique
      {"Holy Green", "Chase"},         // Must visit Holy Green before Chase
      {"Workshop Salon & Boutique", "Chase"}          // Must visit Workshop Salon & Boutique before Chase
  };
  auto result = m.DeliveringTrojan(location_names, dependencies);
  // Define the expected result
  std::vector<std::string> gt = {
      "USC Fisher Museum of Art", "Tommy Trojan", "CAVA", "Holy Green", 
      "Hollywood Wraps", "Haagen Dazs Direct", "Workshop Salon & Boutique", 
      "Chase"  };
  EXPECT_EQ(result, gt);
}

// Test 1 for Queries Function
TEST(TrojanMapTest, Queries) {
  TrojanMap m;
  std::vector<std::pair<double, std::vector<std::string>>> input {{555, {"Tommy Trojan", "GTA VI"}},
                                                                  {10, {"CAVA", "Target"}},
                                                                  {0.02, {"Hollywood Wraps", "Target"}},
                                                                  {999, {"dummy", "dummy"}}};
  auto actual = m.Queries(input);
  std::vector<bool> expected {false, true, false, false};
  EXPECT_EQ(expected, actual);
}
// Test II for Queries Function
TEST(TrojanMapTest, Queries2) {
  TrojanMap m;
  std::vector<std::pair<double, std::vector<std::string>>> input {
                                                                  
                                                                  {11, {" ", "Target"}},
                                                                  {999, {"CAVA", "Mercy please"}}};
  auto actual = m.Queries(input);
  std::vector<bool> expected {false, false};
  EXPECT_EQ(expected, actual);
}

// Test III for Queries Function
TEST(TrojanMapTest, Queries3) {
  TrojanMap m;
  std::vector<std::pair<double, std::vector<std::string>>> input {
                                                                  
                                                                  {11, {" CAVA", " "}},
                                                                  {0.0001, {"CAVA", "Trojan Grounds (Starbucks)"}}};
  auto actual = m.Queries(input);
  std::vector<bool> expected {false, false};
  EXPECT_EQ(expected, actual);
}


// TrojanPath Tests
// performing 3 tests on one function
TEST(TrojanMapTest, TrojanPathTests123) {
    TrojanMap m;

    // test I-> Which Wich?-> New Orleans Fish Market -> Chick-fil-A
    std::vector<std::string> input = {"Which Wich?", "New Orleans Fish Market", "Chick-fil-A"};
    auto path = m.TrojanPath(input);
    std::vector<std::string> gt{"4577908517", "4630604681", "4547476733"};
    EXPECT_EQ(path, gt);

    double calculated_distance = m.CalculatePathLength(path);
    double expected_distance = m.CalculatePathLength(gt);
    std::cout << "Test Case 1: Which Wich? -> New Orleans Fish Market -> Chick-fil-A" << std::endl;
    std::cout << "Calculated Path Length: " << calculated_distance << " miles" << std::endl;
    std::cout << "Expected Path Length: " << expected_distance << " miles" << std::endl;
    EXPECT_NEAR(calculated_distance, expected_distance, 0.001);
  

    // test II New Orleans Fish Market  -> Chick-fil-A -> Which Wich?
    input = {"New Orleans Fish Market", "Chick-fil-A", "Which Wich?"};
    path = m.TrojanPath(input);
    gt = {"4630604681", "4547476733", "4577908517"};
    EXPECT_EQ(path, gt);

    calculated_distance = m.CalculatePathLength(path);
    expected_distance = m.CalculatePathLength(gt);
    std::cout << "Test Case 2: New Orleans Fish Market  -> Chick-fil-A -> Which Wich?" << std::endl;
    std::cout << "Calculated Path Length: " << calculated_distance << " miles" << std::endl;
    std::cout << "Expected Path Length: " << expected_distance << " miles" << std::endl;
    EXPECT_NEAR(calculated_distance, expected_distance, 0.001);

    // test III --> New Orleans Fish Market -> Which Wich? -> Chick-fil-A
    input = {"New Orleans Fish Market", "Which Wich?", "Chick-fil-A"};
    path = m.TrojanPath(input);
    gt = {"4630604681", "4577908517", "4547476733"};
    EXPECT_EQ(path, gt);

    calculated_distance = m.CalculatePathLength(path);
    expected_distance = m.CalculatePathLength(gt);
    std::cout << "Test Case 3: New Orleans Fish Market -> Which Wich? -> Chick-fil-A" << std::endl;
    std::cout << "Calculated Path Length: " << calculated_distance << " miles" << std::endl;
    std::cout << "Expected Path Length: " << expected_distance << " miles" << std::endl;
    EXPECT_NEAR(calculated_distance, expected_distance, 0.001);
}
