/*
 * LRUCache.h
 *
 *  Created on: Jan 27, 2015
 *      Author: chen
 */

#ifndef LRUCACHE_H_
#define LRUCACHE_H_

// A simple LRU cache written in C++
// Hash map + doubly linked list
#include <iostream>
#include <vector>
//#include <ext/hash_map>
#include <unordered_map>
using namespace std;
//using namespace __gnu_cxx;

namespace ns3
{
namespace ndn
{
namespace nrndn
{
namespace cache
{
template <class K, class T>
struct Node{
    K key;
    T data;
    Node *prev, *next;
};

template <class K, class T>
class LRUCache{
public:
    LRUCache(size_t size){
        entries_ = new Node<K,T>[size];
        for(size_t i=0; i<size; ++i)// 瀛樺偍鍙敤缁撶偣鐨勫湴鍧�            free_entries_.push_back(entries_+i);
        head_ = new Node<K,T>;
        tail_ = new Node<K,T>;
        head_->prev = NULL;
        head_->next = tail_;
        tail_->prev = head_;
        tail_->next = NULL;
    }
    ~LRUCache(){
        delete head_;
        delete tail_;
        delete[] entries_;
    }
    void Put(K key, T data){
        Node<K,T> *node = hashmap_[key];
        if(node){ // node exists
            detach(node);
            node->data = data;
            attach(node);
        }
        else{
            if(free_entries_.empty()){// 鍙敤缁撶偣涓虹┖锛屽嵆cache宸叉弧
                node = tail_->prev;
                detach(node);
                hashmap_.erase(node->key);
            }
            else{
                node = free_entries_.back();
                free_entries_.pop_back();
            }
            node->key = key;
            node->data = data;
            hashmap_[key] = node;
            attach(node);
        }
    }
    T Get(K key){
        Node<K,T> *node = hashmap_[key];
        if(node){
            detach(node);
            attach(node);
            return node->data;
        }
        else{// 濡傛灉cache涓病鏈夛紝杩斿洖T鐨勯粯璁ゅ�銆備笌hashmap琛屼负涓�嚧
            return T();
        }
    }
private:
    // 鍒嗙缁撶偣
    void detach(Node<K,T>* node){
        node->prev->next = node->next;
        node->next->prev = node->prev;
    }
    // 灏嗙粨鐐规彃鍏ュご閮�    void attach(Node<K,T>* node){
        node->prev = head_;
        node->next = head_->next;
        head_->next = node;
        node->next->prev = node;
    }
private:
    std::unordered_map<K,Node<K,T>*> hashmap_;
    //hash_map<K, Node<K,T>* > hashmap_;
    vector<Node<K,T>* > free_entries_; // 瀛樺偍鍙敤缁撶偣鐨勫湴鍧�    Node<K,T> *head_, *tail_;
    Node<K,T> *entries_; // 鍙屽悜閾捐〃涓殑缁撶偣
};

} /* namespace cache */
} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* LRUCACHE_H_ */
