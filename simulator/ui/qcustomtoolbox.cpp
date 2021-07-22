#include "ui/qcustomtoolbox.h"

#include <QStyle>
#include <QLayout>
#include <QAbstractButton>
#include <QStyleOptionToolBox>

QSize QCustomToolBox::minimumSizeHint() const {
  QRect toolbox_button_rect = QRect();
  for (auto& child : children()) {
    auto button = dynamic_cast<QAbstractButton*>(child);
    if (button != nullptr) {
      toolbox_button_rect |= button->rect();
      auto option = new QStyleOptionToolBox();
      option->initFrom(button);
      auto rect = this->style()->subElementRect(QStyle::SE_ToolBoxTabContents, option);
      toolbox_button_rect |= rect;
    }
  }
  int extra_height = toolbox_button_rect.height() * count() + layout()->spacing() * count();
  extra_height += 5; // safety margin, although testing shows it works well without it

  auto content_size = this->currentWidget()->sizeHint();
  return QSize(content_size.width(), content_size.height() + extra_height);
}
