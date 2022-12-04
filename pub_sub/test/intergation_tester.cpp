#include <gtest/gtest.h>

#include <listener.h>
#include <talker.h>

class Tester : public Listener {

  public:
    void temp() {
        auto tea = this->get_node_names();
        TEST()
        
    }

};

TEST(Integration_Test, Assert_Check){
    ASSERT_TRUE(true);
}