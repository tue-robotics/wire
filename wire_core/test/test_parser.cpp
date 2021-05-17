#include <gtest/gtest.h>

#include <ros/package.h>

#include <wire/util/ObjectModelParser.h>
#include <wire/storage/KnowledgeDatabase.h>


TEST(ObjectModelParser, parse){
    std::string path = ros::package::getPath("wire_core");
    path += "/models/world_object_models.xml";
    mhf::ObjectModelParser parser(path);

    ASSERT_TRUE(parser.parse(mhf::KnowledgeDatabase::getInstance()));
}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
