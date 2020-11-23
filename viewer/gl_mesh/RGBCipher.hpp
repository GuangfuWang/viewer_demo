#pragma once
#include <glm/glm.hpp>

const glm::vec3 RGBTable[] = {
        //blue: almost no radiation.
        {0.0000f, 0.0000f, 1.0000f},//000  000  255
        {0.1176f, 0.5647f, 1.0000f},//030  144  255
        {0.0000f, 0.7490f, 1.0000f},//000  190  255
        {0.5294f, 0.8078f, 0.9804f},//135  205  250
        {0.5294f, 0.8078f, 0.9216f},//135  205  235
        {0.6902f, 0.8784f, 0.9020f},//176  224  230
        {0.2824f, 0.8196f, 0.8000f},//072  209  204
        {0.0000f, 0.8078f, 0.8196f},//000  205  209
        {0.2510f, 0.8000f, 0.8157f},//064  204  208
        //green: acceptable radaition.
        {0.0000f, 1.0000f, 0.0000f},//000  255  000
        {0.0000f, 0.9333f, 0.4627f},//000  238  118
        {0.0000f, 1.0000f, 0.4980f},//000  255  127
        {0.5647f, 0.9333f, 0.5647f},//144  238  144
        {0.6039f, 1.0000f, 0.6039f},//154  255  154
        {0.3294f, 1.0000f, 0.6235f},//084  255  159
        {0.3059f, 0.9333f, 0.5804f},//078  238  148
        {0.2627f, 0.8039f, 0.5020f},//067  205  128
        {0.7059f, 0.9333f, 0.7059f},//180  238  180
        //yellow: warning amount of radiation.
        {0.6353f, 0.8039f, 0.3529f},//162  205  090
        {0.7373f, 0.9333f, 0.4078f},//188  238  104
        {0.7922f, 1.0000f, 0.4392f},//202  255  112
        {0.7020f, 0.9333f, 0.2275f},//179  238  058
        {0.8039f, 0.8039f, 0.0000f},//205  205  000
        {0.9333f, 0.9333f, 0.0000f},//238  238  000
        {1.0000f, 1.0000f, 0.0000f},//255  255  000
        {0.9333f, 0.9098f, 0.6667f},//238  232  170
        {1.0000f, 0.7569f, 0.7569f},//255  193  193
        //red: danger level of radiation.
        {0.9333f, 0.7059f, 0.7059f},//238  180  180
        {0.8039f, 0.6078f, 0.6078f},//205  155  155
        {0.9804f, 0.5020f, 0.4471f},//250  128  114
        {1.0000f, 0.4157f, 0.4157f},//255  106  106
        {0.9333f, 0.3882f, 0.3882f},//238  099  099
        {0.8039f, 0.3333f, 0.3333f},//205  085  085
        {1.0000f, 0.2706f, 0.0000f},//255  069  000
        {1.0000f, 0.0000f, 0.0000f},//255  000  000
        {0.6980f, 0.1333f, 0.1333f},//178  034  034

};

