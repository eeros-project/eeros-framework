#ifndef ORG_EEROS_UI_BASEUI_HPP_
#define ORG_EEROS_UI_BASEUI_HPP_

namespace eeros {
	namespace ui {
		
		class BaseUI {
		
		public:
			virtual void addMessage(unsigned level, std::string message) = 0;
			
		}; // class BaseUI
	}; // namespace ui
}; // namespace eeros

#endif // ORG_EEROS_UI_BASEUI_HPP_
