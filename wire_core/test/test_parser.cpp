#include <gtest/gtest.h>

#include <ros/package.h>

#include <wire/util/ObjectModelParser.h>
#include <wire/storage/KnowledgeDatabase.h>


TEST(ObjectModelParser, parse){
    std::string path = ros::package::getPath("wire_core");
    path += "/test/test_world_object_models.xml";
    std::cout << "Parsing: " << path << std::endl;
    mhf::ObjectModelParser parser(path, "plugin_test");

    ASSERT_TRUE(parser.parse(mhf::KnowledgeDatabase::getInstance())) << parser.getErrorMessage();
}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
