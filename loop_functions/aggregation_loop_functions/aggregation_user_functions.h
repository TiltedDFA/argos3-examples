#ifndef AGGREGATION_USER_FUNCTION_H
#define AGGREGATION_USER_FUNCTION_H
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "controllers/footbot_aggregation_one/footbot_aggregation_one.h"


class CAggergationUserFunction : public argos::CQTOpenGLUserFunctions {

public:

   CAggergationUserFunction();

   virtual ~CAggergationUserFunction() {}

   void Draw(argos::CFootBotEntity& c_entity);
   
};
#endif // #ifndef AGGREGATION_USER_FUNCTION_H