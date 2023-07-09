#include "aggregation_user_functions.h"

/****************************************/
/****************************************/

CAggergationUserFunction::CAggergationUserFunction() {
   RegisterUserFunction<CAggergationUserFunction,argos::CFootBotEntity>(&CAggergationUserFunction::Draw);
}

/****************************************/
/****************************************/

void CAggergationUserFunction::Draw(argos::CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   QFont font = QFont("SansSerif", 20, QFont::Bold);
   CFootBotAggregationOne & controller = dynamic_cast<CFootBotAggregationOne&>(c_entity.GetControllableEntity().GetController());
   std::string text = c_entity.GetId() + ": " + controller.GetHopCount();

   DrawText(argos::CVector3(0.0, 0.0, 0.3),   // position
            text.c_str(),
            argos::CColor::GREEN,
            font); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAggergationUserFunction, "aggregation_user_function")
