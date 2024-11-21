#include <iostream>
#include <random>
#include <cmath>
using namespace std;

class Vertex {
public:
    bool m_isMetroStation;
    int m_id;

    Vertex(bool isMetroStation, int id)
        : m_isMetroStation(isMetroStation), m_id(id) {}

    bool isMetroStation() {
        return m_isMetroStation;
    }

    int id() {
        return m_id;
    }
};

class Edge {
public:
    Vertex* m_vertex1;  
    Vertex* m_vertex2;  
    int m_distance;
    int m_zipCode;
    int m_street;
    float m_trafficRate;
    float m_excavationCost;

    Edge(int distance, Vertex* vertex1, Vertex* vertex2, float trafficRate)
        : m_distance(distance), m_vertex1(vertex1), m_vertex2(vertex2), m_trafficRate(trafficRate) {
        
        m_excavationCost = distance / 2.0f;
        m_zipCode = generateRandomCode(4);  
        m_street = generateRandomCode(8);  
    }

    int generateRandomCode(int numDigits) {
        random_device rd;  
        mt19937 gen(rd());  
        uniform_int_distribution<int> dist(pow(10, numDigits - 1), pow(10, numDigits) - 1);  
        return dist(gen);  
    }

    Vertex* vertex1() {
        return m_vertex1;
    }

    Vertex* vertex2() {
        return m_vertex2;
    }

    int distance() {
        return m_distance;
    }

    int id_zipCode() {
        return m_zipCode;
    }

    int id_street() {
        return m_street;
    }

    float trafficRate() {
        return m_trafficRate;
    }

    float excavationCost() {
        return m_excavationCost;
    }
};
