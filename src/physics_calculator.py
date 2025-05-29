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


#ifndef INVENTORY_MANAGEMENT_H
#define INVENTORY_MANAGEMENT_H

#include "base_class.h"
#include <vector>

class PartsOrder : public Order {
private:
    Part part;
    int price_per_id;
public:
    PartsOrder(Part part, int price);
    void process(Inventory& inventory) override;
};

class SellOrder : public Order {
private:
    Item item;
public:
    SellOrder(Item item);
    void process(Inventory& inventory) override;
};

class InventoryManagement {
private:
    Inventory inventory;
    std::vector<Order*> orders;
public:
    InventoryManagement();
    ~InventoryManagement();
    void PurchaseParts(Part part, int price_per_id);
    void SellItem(Item item);
    void ProcessAll();
};

#endif




#include "inventory_management.h"

InventoryManagement::InventoryManagement() {
    inventory.fund = 0;
}

InventoryManagement::~InventoryManagement() {
    for (Order* order : orders) {
        delete order;
    }
}

PartsOrder::PartsOrder(Part part, int price) : part(part), price_per_id(price) {}

void PartsOrder::process(Inventory& inventory) {
    int total_cost = part.count * price_per_id;
    inventory.fund -= total_cost;
    bool found = false;
    for (auto& invPart : inventory.parts) {
        if (invPart.id == part.id) {
            invPart.count += part.count;
            found = true;
            break;
        }
    }
    if (!found) {
        inventory.parts.push_back(part);
    }
}

SellOrder::SellOrder(Item item) : item(item) {}

void SellOrder::process(Inventory& inventory) {
    for (const auto& required : item.parts) {
        bool found = false;
        for (const auto& invPart : inventory.parts) {
            if (invPart.id == required.id) {
                if (invPart.count < required.count) {
                    return;
                }
                found = true;
                break;
            }
        }
        if (!found) {
            return;
        }
    }
    for (const auto& required : item.parts) {
        for (auto& invPart : inventory.parts) {
            if (invPart.id == required.id) {
                invPart.count -= required.count;
                break;
            }
        }
    }
    inventory.fund += item.price;
}

void InventoryManagement::PurchaseParts(Part part, int price_per_id) {
    orders.push_back(new PartsOrder(part, price_per_id));
}

void InventoryManagement::SellItem(Item item) {
    orders.push_back(new SellOrder(item));
}

void InventoryManagement::ProcessAll() {
    for (Order* order : orders) {
        order->process(inventory);
        delete order;
    }
    orders.clear();
}

#include "arena.h"
#include <vector>

Arena::Arena(Node* team_a, Node* team_b) {
    Node *a_sub0 = nullptr, *a_sub1 = nullptr, *a_sub2 = nullptr;
    Node *a_tail0 = nullptr, *a_tail1 = nullptr, *a_tail2 = nullptr;
    Node* current = team_a;
    int idx = 0;
    while (current) {
        Node* next = current->next;
        current->next = nullptr;
        int r = idx % 3;
        if (r == 0) {
            if (!a_sub0) {
                a_sub0 = current;
                a_tail0 = current;
            } else {
                a_tail0->next = current;
                a_tail0 = current;
            }
        } else if (r == 1) {
            if (!a_sub1) {
                a_sub1 = current;
                a_tail1 = current;
            } else {
                a_tail1->next = current;
                a_tail1 = current;
            }
        } else {
            if (!a_sub2) {
                a_sub2 = current;
                a_tail2 = current;
            } else {
                a_tail2->next = current;
                a_tail2 = current;
            }
        }
        current = next;
        idx++;
    }
    a_sub0 = reverse(a_sub0);
    a_sub1 = reverse(a_sub1);
    a_sub2 = reverse(a_sub2);
    lineup_a_.clear();
    lineup_a_.push_back(a_sub0);
    lineup_a_.push_back(a_sub1);
    lineup_a_.push_back(a_sub2);

    Node *b_sub0 = nullptr, *b_sub1 = nullptr, *b_sub2 = nullptr;
    Node *b_tail0 = nullptr, *b_tail1 = nullptr, *b_tail2 = nullptr;
    current = team_b;
    idx = 0;
    while (current) {
        Node* next = current->next;
        current->next = nullptr;
        int r = idx % 3;
        if (r == 0) {
            if (!b_sub0) {
                b_sub0 = current;
                b_tail0 = current;
            } else {
                b_tail0->next = current;
                b_tail0 = current;
            }
        } else if (r == 1) {
            if (!b_sub1) {
                b_sub1 = current;
                b_tail1 = current;
            } else {
                b_tail1->next = current;
                b_tail1 = current;
            }
        } else {
            if (!b_sub2) {
                b_sub2 = current;
                b_tail2 = current;
            } else {
                b_tail2->next = current;
                b_tail2 = current;
            }
        }
        current = next;
        idx++;
    }
    b_sub0 = reverse(b_sub0);
    b_sub1 = reverse(b_sub1);
    b_sub2 = reverse(b_sub2);
    lineup_b_.clear();
    lineup_b_.push_back(b_sub0);
    lineup_b_.push_back(b_sub1);
    lineup_b_.push_back(b_sub2);
}

size_t Arena::length_(Node* head) {
    size_t len = 0;
    while (head) {
        len++;
        head = head->next;
    }
    return len;
}

Node* Arena::reverse(Node* head) {
    Node* prev = nullptr;
    Node* cur = head;
    while (cur) {
        Node* nxt = cur->next;
        cur->next = prev;
        prev = cur;
        cur = nxt;
    }
    return prev;
}

std::vector<int> Arena::GetDuelResult() {
    std::vector<int> results(3);
    for (int i = 0; i < 3; i++) {
        Node* a_head = lineup_a_[i];
        Node* b_head = lineup_b_[i];
        Node* a_copy = nullptr;
        Node* b_copy = nullptr;
        if (a_head) {
            a_copy = new Node(a_head->value);
            Node* curNew = a_copy;
            Node* curOld = a_head->next;
            while (curOld) {
                curNew->next = new Node(curOld->value);
                curNew = curNew->next;
                curOld = curOld->next;
            }
            curNew->next = nullptr;
        }
        if (b_head) {
            b_copy = new Node(b_head->value);
            Node* curNew = b_copy;
            Node* curOld = b_head->next;
            while (curOld) {
                curNew->next = new Node(curOld->value);
                curNew = curNew->next;
                curOld = curOld->next;
            }
            curNew->next = nullptr;
        }
        Node* curA = a_copy;
        Node* curB = b_copy;
        bool stop = false;
        while (curA && curB && !stop) {
            Node* A = curA;
            Node* B = curB;
            int rA = 0, rB = 0;
            while (true) {
                int valA = A->value;
                int valB = B->value;
                A->value = valA - valB;
                B->value = valB - valA;
                rA++;
                rB++;
                if (A->value <= 0 && B->value <= 0) {
                    curA = A->next;
                    curB = B->next;
                    delete A;
                    delete B;
                    break;
                }
                if (A->value > 0 && B->value <= 0) {
                    curB = B->next;
                    delete B;
                    if (rA < 2 && curB) {
                        B = curB;
                        continue;
                    } else {
                        break;
                    }
                }
                if (B->value > 0 && A->value <= 0) {
                    curA = A->next;
                    delete A;
                    if (rB < 2 && curA) {
                        A = curA;
                        continue;
                    } else {
                        break;
                    }
                }
                if (A->value > 0 && B->value > 0) {
                    if (rA < 2 && rB < 2) {
                        continue;
                    } else {
                        stop = true;
                        break;
                    }
                }
            }
        }
        int countA = 0;
        int countB = 0;
        Node* temp = curA;
        while (temp) {
            countA++;
            temp = temp->next;
        }
        temp = curB;
        while (temp) {
            countB++;
            temp = temp->next;
        }
        results[i] = countA - countB;
        temp = curA;
        while (temp) {
            Node* nxt = temp->next;
            delete temp;
            temp = nxt;
        }
        temp = curB;
        while (temp) {
            Node* nxt = temp->next;
            delete temp;
            temp = nxt;
        }
    }
    return results;
}

[ RUN      ] Arena.Advance
C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 15, 15, 16 }
  b
    Which is: { 16, 16, 16 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { -47, -47, -47 }
  b
    Which is: { -56, -56, -56 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 98, 99, 99 }
  b
    Which is: { 116, 117, 117 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 34, 34, 35 }
  b
    Which is: { 37, 37, 37 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 5, 6, 6 }
  b
    Which is: { 6, 6, 6 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 35, 35, 35 }
  b
    Which is: { 37, 37, 38 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 37, 37, 37 }
  b
    Which is: { 38, 38, 38 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { 30, 31, 31 }
  b
    Which is: { 33, 33, 33 }

C:\Users\FIBO-1\Documents\Dav\oop8-silver-xXxMrDEDOxXx\test\quiz_test.cpp:70: Failure
Expected equality of these values:
  a
    Which is: { -16, -17, -17 }
  b
    Which is: { -17, -17, -17 }

[  FAILED  ] Arena.Advance (3 ms)
[----------] 3 tests from Arena (3 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 1 test suite ran. (3 ms total)
[  PASSED  ] 2 tests.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] Arena.Advance



