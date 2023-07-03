#include "aggregation_user_functions.h"

/****************************************/
/****************************************/

CAggergationUserFunction::CAggergationUserFunction() {
   RegisterUserFunction<CAggergationUserFunction,CFootBotEntity>(&CAggergationUserFunction::Draw);
}

/****************************************/
/****************************************/

void CAggergationUserFunction::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   QFont font = QFont("SansSerif", 50, QFont::Bold);
   argos::CCI_Controller& controller = c_entity.GetControllableEntity().GetController();
   std::string text = c_entity.GetId() + ": " + std::to_string(dynamic_cast<CFootBotAggregationOne&>(controller).hop_count);

   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            text.c_str(),
            CColor::BLUE,
            font); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAggergationUserFunction, "aggregation_user_function")
