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
   QFont font = QFont("SansSerif", 20, QFont::Bold);
   CFootBotAggregationOne & controller = dynamic_cast<CFootBotAggregationOne&>(c_entity.GetControllableEntity().GetController());
   std::string text = c_entity.GetId() + ": " + controller.GetHopCount();

   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            text.c_str(),
            CColor::GREEN,
            font); // text
   /*argos::CCI_Controller& controller = c_entity.GetControllableEntity().GetController();
   DrawText(CVector3(0.0, -0.05, 0.3),   // position
            c_entity.GetId().c_str(),
            CColor::RED); // text

   DrawText(CVector3(0.0, 0.05, 0.3),   // position
            //dynamic_cast<CFootBotAggregationOne&>(c_entity) CFootBotAggregationOne is controller
            dynamic_cast<CFootBotAggregationOne&>(controller)
                  .GetHopCount()
                  .c_str(),
            CColor::RED); // text
   */
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAggergationUserFunction, "aggregation_user_function")
