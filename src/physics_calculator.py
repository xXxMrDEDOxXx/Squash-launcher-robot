class PhysicsCalculator:
    """
    Contains methods for physics calculations related to pressure control.
    This could include formulas for pressure, volume, flow, etc. based on domain requirements.
    """
    def __init__(self, launch_params: LaunchParams = None):
        # If any constants or environment parameters are needed (from LaunchParams), store them.
        self.launch_params = launch_params
        # For example, if we needed temperature or volume from LaunchParams, we could store them here.
    
    def compute_recommended_pressure(self, actual_pressure: float) -> float:
        """
        Compute a recommended pressure based on the current actual pressure and target pressure.
        This is a placeholder for the real physics-based recommendation formula.
        """
        if self.launch_params is not None:
            target = getattr(self.launch_params, "target_pressure", None)
        else:
            target = None

        if target is None:
            # No target provided, just return the actual pressure as no change recommended
            return actual_pressure
        
        # Example strategy: simple proportional adjustment towards target pressure
        recommended = actual_pressure
        # If actual is below target, recommend a slight increase; if above, slight decrease.
        # (In a real scenario, this could be a PID controller or a complex physics formula.)
        diff = target - actual_pressure
        # Apply a fraction of the difference as recommendation (to avoid sudden jumps)
        recommended += 0.5 * diff  # move halfway towards target for demo
        return recommended
    
    # Additional physics calculation methods could be added here, e.g.:
    # def calculate_pressure_drop(self, flow_rate, pipe_length, diameter): ...
    # def needed_pressure_for_volume(self, volume, temperature): ...
Arena::Arena(Node* team_a, Node* team_b) {
    Node* a0 = nullptr;
    Node* a1 = nullptr;
    Node* a2 = nullptr;
    Node* cur = team_a;
    int idx = 0;
    while(cur) {
        Node* next = cur->next;
        if(idx % 3 == 0) {
            cur->next = a0;
            a0 = cur;
        } else if(idx % 3 == 1) {
            cur->next = a1;
            a1 = cur;
        } else {
            cur->next = a2;
            a2 = cur;
        }
        cur = next;
        idx++;
    }
    lineup_a_[0] = a0;
    lineup_a_[1] = a1;
    lineup_a_[2] = a2;
    Node* b0 = nullptr;
    Node* b1 = nullptr;
    Node* b2 = nullptr;
    cur = team_b;
    idx = 0;
    while(cur) {
        Node* next = cur->next;
        if(idx % 3 == 0) {
            cur->next = b0;
            b0 = cur;
        } else if(idx % 3 == 1) {
            cur->next = b1;
            b1 = cur;
        } else {
            cur->next = b2;
            b2 = cur;
        }
        cur = next;
        idx++;
    }
    lineup_b_[0] = b0;
    lineup_b_[1] = b1;
    lineup_b_[2] = b2;
}
std::vector<int> Arena::GetDuelResult() {
    std::vector<int> result(3);
    for(int i = 0; i < 3; i++) {
        Node* a = lineup_a_[i];
        Node* b = lineup_b_[i];
        int survivorsA = 0;
        int survivorsB = 0;
        for(Node* tmp = a; tmp; tmp = tmp->next) survivorsA++;
        for(Node* tmp = b; tmp; tmp = tmp->next) survivorsB++;
        while(a && b) {
            int winsA = 0;
            int winsB = 0;
            while(a && b && winsA < 2 && winsB < 2) {
                if(a->value == b->value) {
                    survivorsA--;
                    survivorsB--;
                    a = a->next;
                    b = b->next;
                    break;
                } else if(a->value > b->value) {
                    a->value -= b->value;
                    survivorsB--;
                    b = b->next;
                    winsA++;
                    if(winsA == 2) {
                        a = a->next;
                        break;
                    }
                } else {
                    b->value -= a->value;
                    survivorsA--;
                    a = a->next;
                    winsB++;
                    if(winsB == 2) {
                        b = b->next;
                        break;
                    }
                }
            }
        }
        result[i] = survivorsA - survivorsB;
    }
    return result;
}



#ifndef INVENTORY_MANAGEMENT_H
#define INVENTORY_MANAGEMENT_H

#include <vector>
#include <map>
#include "base_class.h"

class InventoryManagement;

class PartsOrder : public Order {
private:
    Part part;
    int price_per_id;
public:
    PartsOrder(Part part, int price_per_id);
    void Process(InventoryManagement *inv) override;
};

class ItemOrder : public Order {
private:
    Item item;
public:
    ItemOrder(Item item);
    void Process(InventoryManagement *inv) override;
};

class InventoryManagement {
public:
    InventoryManagement();
    void PurchaseParts(Part part, int price_per_id);
    void SellItem(Item item);
    void ProcessAll();
    int funds;
    std::map<int,int> inventory;
private:
    std::vector<Order*> orders;
};

#endif



#include "inventory_management.h"

PartsOrder::PartsOrder(Part part, int price_per_id) : part(part), price_per_id(price_per_id) {}

void PartsOrder::Process(InventoryManagement *inv) {
    int totalCost = part.quantity * price_per_id;
    if (inv->funds >= totalCost) {
        inv->funds -= totalCost;
        inv->inventory[part.id] += part.quantity;
    }
}

ItemOrder::ItemOrder(Item item) : item(item) {}

void ItemOrder::Process(InventoryManagement *inv) {
    for (const Part &p : item.GetParts()) {
        if (inv->inventory[p.id] < p.quantity) {
            return;
        }
    }
    for (const Part &p : item.GetParts()) {
        inv->inventory[p.id] -= p.quantity;
    }
    inv->funds += item.GetPrice();
}

InventoryManagement::InventoryManagement() : funds(0) {}

void InventoryManagement::PurchaseParts(Part part, int price_per_id) {
    orders.push_back(new PartsOrder(part, price_per_id));
}

void InventoryManagement::SellItem(Item item) {
    orders.push_back(new ItemOrder(item));
}

void InventoryManagement::ProcessAll() {
    for (Order* order : orders) {
        order->Process(this);
        delete order;
    }
    orders.clear();
}



