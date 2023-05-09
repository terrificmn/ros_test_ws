#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>


// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}


class SaySomething : public BT::SyncActionNode {
Public:
    SaySomething(const std::string& name, const BT::NodeConfiguration & config) 
        : BT::SyncActionNode(name, config) {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override {
        auto msg = getInput<std::string>("message");
        std::cout << "Robot says: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    //mendatory to define this staic method
    static BT::PortsList providePorts() {
        return (BT::InputPort<std::string>("message"));
    }
};

int main() {
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    auto tree = factory.createTreeFromFile("bt_tree.xml");

}