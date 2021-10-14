#pragma once

#include <JuceHeader.h>

inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

struct Node {
    float velocity     = 0.f;
    float lateral_displacement = 0.f;
    static constexpr float mass = 0.1f;
    
    inline void integrateSpringForce(float spring_constant, float spring_other_end, float dt) {
        const float x =  spring_other_end - lateral_displacement;
        const float F = x * spring_constant;
        const float a = F / mass;
        velocity += a * dt;
    }
    
    inline void updateDisplacement(float dt) {
        lateral_displacement += velocity * dt;
    }
};


/**
 * String is modelled as a sequence of nodes connected by springs
 */
template<int NUMNODES>
class String {
public:
    inline void step(float dt) {
        // In this initial simple model, only lateral displacement is considered
        nodes[0].integrateSpringForce(spring_constant, 0.f, dt);  // The static point at the end of the string
        if(NUMNODES > 1)
            nodes[0].integrateSpringForce(spring_constant, nodes[1].lateral_displacement, dt);
        
        for(auto i = 1; i < (NUMNODES-1); ++i) {
            nodes[i].integrateSpringForce(spring_constant, nodes[i-1].lateral_displacement, dt);
            nodes[i].integrateSpringForce(spring_constant, nodes[i+1].lateral_displacement, dt);
        }
        
        if(NUMNODES > 1)
            nodes[NUMNODES-1].integrateSpringForce(spring_constant, nodes[NUMNODES-2].lateral_displacement, dt);
        nodes[NUMNODES-1].integrateSpringForce(spring_constant, 0.f, dt);   // The static point at the end of the string
        
        for(auto i = 0; i < NUMNODES; ++i) {
            nodes[i].updateDisplacement(dt);
        }
    }
    
    inline float sample() {
        // There are lots of ways to do this. This is probably the stupidest.
        return nodes[NUMNODES/2].lateral_displacement;
    }
    
    /**
     * This could stand to be a lot more sophisticated but it will do for a start
     * @param position Ratio representing distance along string to pluck at
     * @param strength The amount to displace the string by
     */
    inline void pluck(float position, float strength) {
        assert(position >= 0 && position <= 1);
        int start_index = int(float(NUMNODES - 1) * position);
        
        // displace the string in a triangle shape with it's peak at the point of plucking
        for(int i = start_index; i < NUMNODES; ++i) {
            const float distance_to_end = float(NUMNODES - i) / float(NUMNODES - start_index);
            nodes[i].lateral_displacement = lerp(0.f, strength, distance_to_end);
            nodes[i].velocity = 0.f;
        }
        for(int i = 0; i < start_index; ++i) {
            const float distance_to_start = float(start_index) / float(start_index - i);
            nodes[i].lateral_displacement = lerp(0.f, strength, distance_to_start);
            nodes[i].velocity = 0.f;
        }
    }
private:
    Node nodes[NUMNODES];
    const float length = 1.f;
    const float spring_constant = 100000.f;
};


//==============================================================================
/*
    This component lives inside our window, and this is where you should put all
    your controls and content.
*/
class MainComponent  : public juce::AudioAppComponent
{
public:
    //==============================================================================
    MainComponent();
    ~MainComponent() override;

    //==============================================================================
    void prepareToPlay (int samplesPerBlockExpected, double sampleRate) override;
    void getNextAudioBlock (const juce::AudioSourceChannelInfo& bufferToFill) override;
    void releaseResources() override;

    //==============================================================================
    void paint (juce::Graphics& g) override;
    void resized() override;

private:
    //==============================================================================
    // Your private member variables go here...
    const float level = 0.5f;
    
    String<1600> string;
    float timestep = 0.f;
//    const float mass = 1.f;
//    const float k = 1000000.f;
//    float displacement = 1.f;
//    float velocity = 0.f;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
