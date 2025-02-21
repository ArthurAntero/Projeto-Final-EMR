<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <ReactiveSequence>
            <Fallback>
                <Condition ID="IsBatteryAbove15" battery_level="{battery_level}"/>
                <Action ID="GoToRechargeBase" recharge_base_x="{recharge_base_x}" recharge_base_y="{recharge_base_y}" recharge_base_yaw="{recharge_base_yaw}"/>
            </Fallback>
            <Sequence name="PickupMeal">
                <Action ID="GoToKitchen" kitchen_x="{kitchen_x}" kitchen_y="{kitchen_y}" kitchen_yaw="{kitchen_yaw}"/>
                <Action ID="PickUpMeal" meals="{meals}" meals_quantity="{meals_quantity}"/>
            </Sequence>
            <Repeat num_cycles="{meals_quantity}">
                <Sequence name="delivery">
                    <Action ID="FetchRoom" meals="{meals}" room_positions="{room_positions}" room_x="{room_x}" room_y="{room_y}" room_yaw="{room_yaw}"/>
                    <Action ID="GoToRoom" room_x="{room_x}" room_y="{room_y}" room_yaw="{room_yaw}"/>
                    <Fallback>
                        <Sequence>
                            <Condition ID="IsPickUpAvailable" available="{available}" meals="{meals}"/>
                            <Action ID="WaitForPickUp" pickup_time_sec="{pickup_time_sec}"/>
                        </Sequence>
                        <Action ID="DeliverToTable"/>
                    </Fallback>
                    <Condition ID="GotRightMeal" meals="{meals}" room_positions="{room_positions}"/>
                    <Action ID="GoToWaitingRoom" waiting_room_x="{waiting_room_x}" waiting_room_y="{waiting_room_y}" waiting_room_yaw="{waiting_room_yaw}"/>
                </Sequence>
            </Repeat>
            <Sequence name="PickUpDishes">
                <Action ID="WaitForCall" dishes="{dishes}" dishes_quantity="{dishes_quantity}"/>
                <Repeat num_cycles="{dishes_quantity}">
                    <Sequence>
                        <Action ID="FetchRoomForDishes" dishes="{dishes}" room_positions="{room_positions}" room_x="{room_x}" room_y="{room_y}" room_yaw="{room_yaw}"/>
                        <Action ID="GoToRoom" room_x="{room_x}" room_y="{room_y}" room_yaw="{room_yaw}"/>
                        <Action ID="PickUpDish" dishes="{dishes}"/>
                    </Sequence>
                </Repeat>
                <Action ID="GoToKitchen" kitchen_x="{kitchen_x}" kitchen_y="{kitchen_y}" kitchen_yaw="{kitchen_yaw}"/>
                <Action ID="DeliverDishesToKitchen" dishes="{dishes}"/>
            </Sequence>
        </ReactiveSequence>
    </BehaviorTree>
    <!-- ////////// -->
    <TreeNodesModel>
        <Action ID="CheckBattery"/>
        <Action ID="DeliverDishesToKitchen">
            <input_port name="dishes"/>
        </Action>
        <Action ID="DeliverToTable"/>
        <Action ID="FetchRoom">
            <input_port name="meals"/>
            <input_port name="room_positions"/>
            <output_port name="room_x"/>
            <output_port name="room_y"/>
            <output_port name="room_yaw"/>
        </Action>
        <Action ID="FetchRoomForDishes">
            <input_port name="dishes"/>
            <input_port name="room_positions"/>
            <output_port name="room_x"/>
            <output_port name="room_y"/>
            <output_port name="room_yaw"/>
        </Action>
        <Action ID="GoToKitchen">
            <input_port name="kitchen_x"/>
            <input_port name="kitchen_y"/>
            <input_port name="kitchen_yaw"/>
        </Action>
        <Action ID="GoToRechargeBase">
            <input_port name="recharge_base_x"/>
            <input_port name="recharge_base_y"/>
            <input_port name="recharge_base_yaw"/>
        </Action>
        <Action ID="GoToRoom">
            <input_port name="room_x"/>
            <input_port name="room_y"/>
            <input_port name="room_yaw"/>
        </Action>
        <Action ID="GoToWaitingRoom">
            <input_port name="waiting_room_x"/>
            <input_port name="waiting_room_y"/>
            <input_port name="waiting_room_yaw"/>
        </Action>
        <Condition ID="GotRightMeal">
            <input_port name="meals"/>
            <input_port name="room_positions"/>
        </Condition>
        <Condition ID="IsBatteryAbove15">
            <output_port name="battery_level"/>
        </Condition>
        <Condition ID="IsPickUpAvailable">
            <input_port name="available"/>
            <input_port name="meals"/>
        </Condition>
        <Action ID="PickUpDish">
            <input_port name="dishes"/>
        </Action>
        <Action ID="PickUpMeal">
            <output_port name="meals"/>
            <output_port name="meals_quantity"/>
        </Action>
        <Action ID="WaitForCall">
            <output_port name="dishes"/>
            <output_port name="dishes_quantity"/>
        </Action>
        <Action ID="WaitForPickUp">
            <input_port name="pickup_time_sec"/>
        </Action>
    </TreeNodesModel>
    <!-- ////////// -->
</root>