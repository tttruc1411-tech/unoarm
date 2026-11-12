// UNOARM v3 (ORC CONTROL HUB): giu nguyen 8 khoi va noi dung chu cua ban xbot (kdi_xbot_extension_arm).
// Khac ban xbot o phan sinh code: ham di chuyen cua kdi_unoarm.py la async nen sinh `await`,
// va moi khoi dang ky kdi_unoarm.dung_lai() vao ham deinit() cua Hub (chay khi chuong trinh bi dung).
// Khong co khoi khoi tao va khong co nut BOOT (giong ban xbot): lan di chuyen dau tien thu vien tu noi servo roi di.
var ARM_COLOR = "#3d64bf";

function unoarmUse() {
  Blockly.Python.definitions_['import_kdi_unoarm'] = 'import kdi_unoarm';
  Blockly.Python.definitions_['deinit_kdi_unoarm'] = 'kdi_unoarm.dung_lai()';
}

// Định nghĩa block: Set dimensions
Blockly.Blocks['unoarm_set_dimensions'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Set dimensions");
    this.appendValueInput("D1")
        .setCheck("Number")
        .appendField("d1");
    this.appendValueInput("D2")
        .setCheck("Number")
        .appendField("d2");
    this.appendValueInput("D3")
        .setCheck("Number")
        .appendField("d3");
    this.appendValueInput("D4")
        .setCheck("Number")
        .appendField("d4");
    this.appendValueInput("D5")
        .setCheck("Number")
        .appendField("d5");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Set the five dimension parameters d1 to d5");
    this.setHelpUrl("");
  }
};

// Định nghĩa block: Check valid angles S2, S3 (value block)
Blockly.Blocks['unoarm_check_valid'] = {
  init: function() {
    // this.appendValueInput("S2")
    //     .setCheck("Number")
    //     .appendField("Check valid S2");
    // this.appendValueInput("S3")
    //     .setCheck("Number")
    //     .appendField("S3");
    this.appendDummyInput()
        .appendField("Kiểm tra tính hợp lệ của góc S2:");
    this.appendValueInput("S2")
        .setCheck("Number");
    this.appendDummyInput()
        .appendField("S3:");
    this.appendValueInput("S3")
        .setCheck("Number");
    this.setInputsInline(true);
    // Quan trọng: khai báo là output block và kiểu Boolean
    this.setOutput(true, "Boolean");
    this.setColour(ARM_COLOR);
    this.setTooltip("Returns true if the (S2, S3) angles are within valid range");
    this.setHelpUrl("");
  }
};

// Generator cho block Set dimensions
Blockly.Python['unoarm_set_dimensions'] = function(block) {
  unoarmUse();
  var d1 = Blockly.Python.valueToCode(block, 'D1', Blockly.Python.ORDER_ATOMIC) || '0';
  var d2 = Blockly.Python.valueToCode(block, 'D2', Blockly.Python.ORDER_ATOMIC) || '0';
  var d3 = Blockly.Python.valueToCode(block, 'D3', Blockly.Python.ORDER_ATOMIC) || '0';
  var d4 = Blockly.Python.valueToCode(block, 'D4', Blockly.Python.ORDER_ATOMIC) || '0';
  var d5 = Blockly.Python.valueToCode(block, 'D5', Blockly.Python.ORDER_ATOMIC) || '0';
  return 'kdi_unoarm.set_dimensions(' + d1 + ', ' + d2 + ', ' + d3 + ', ' + d4 + ', ' + d5 + ')\n';
};

// Generator cho block Check valid S2, S3
Blockly.Python['unoarm_check_valid'] = function(block) {
  unoarmUse();
  var s2 = Blockly.Python.valueToCode(block, 'S2', Blockly.Python.ORDER_ATOMIC) || '0';
  var s3 = Blockly.Python.valueToCode(block, 'S3', Blockly.Python.ORDER_ATOMIC) || '0';
  var code = 'kdi_unoarm.kiem_tra_tinh_hop_le(' + s2 + ', ' + s3 + ')';
  return [code, Blockly.Python.ORDER_FUNCTION_CALL];
};

// 1) Block: Go to S2, S3
Blockly.Blocks['unoarm_go_to_s2_s3'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Đi đến góc S2:");
    this.appendValueInput("TO_S2")
        .setCheck("Number");
    this.appendDummyInput()
        .appendField("S3:");
    this.appendValueInput("TO_S3")
        .setCheck("Number");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Move the arm to the specified S2 and S3 angles");
    this.setHelpUrl("");
  }
};

// 2) Python generator cho block Go to S2, S3
Blockly.Python['unoarm_go_to_s2_s3'] = function(block) {
  unoarmUse();

  var toS2 = Blockly.Python.valueToCode(block, 'TO_S2', Blockly.Python.ORDER_ATOMIC) || '0';
  var toS3 = Blockly.Python.valueToCode(block, 'TO_S3', Blockly.Python.ORDER_ATOMIC) || '0';
  return 'await kdi_unoarm.go_to_S2_S3(' + toS2 + ', ' + toS3 + ')\n';
};

// 1. Định nghĩa block UNOARM_GO_TO_R_H
Blockly.Blocks['unoarm_go_to_r_h'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Di chuyển đến r:");
    this.appendValueInput("R")
        .setCheck("Number");
    this.appendDummyInput()
        .appendField("h:");
    this.appendValueInput("H")
        .setCheck("Number");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Di chuyển UnoArm đến toạ độ (r, h)");
    this.setHelpUrl("");
  }
};

// 2. Python generator cho block UNOARM_GO_TO_R_H
Blockly.Python['unoarm_go_to_r_h'] = function(block) {
  var r = Blockly.Python.valueToCode(block, 'R', Blockly.Python.ORDER_ATOMIC) || '0';
  var h = Blockly.Python.valueToCode(block, 'H', Blockly.Python.ORDER_ATOMIC) || '0';
  unoarmUse();
  var code = 'await kdi_unoarm.go_to_r_h(' + r + ', ' + h + ')\n';
  return code;
};

// --- Block: Get current r_now (Number) ---
Blockly.Blocks['unoarm_get_r_now'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Giá trị r hiện tại");
    this.setOutput(true, "Number");
    this.setColour(ARM_COLOR);
    this.setTooltip("Trả về giá trị r_now hiện tại");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_get_r_now'] = function(block) {
  unoarmUse();
  return ['kdi_unoarm.r_now', Blockly.Python.ORDER_ATOMIC];
};

// --- Block: Get current h_now (Number) ---
Blockly.Blocks['unoarm_get_h_now'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Giá trị h hiện tại");
    this.setOutput(true, "Number");
    this.setColour(ARM_COLOR);
    this.setTooltip("Trả về giá trị h_now hiện tại");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_get_h_now'] = function(block) {
  unoarmUse();
  return ['kdi_unoarm.h_now', Blockly.Python.ORDER_ATOMIC];
};

// --- Block: Get current S2 (Number) ---
Blockly.Blocks['unoarm_get_S2'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Giá trị S2 hiện tại");
    this.setOutput(true, "Number");
    this.setColour(ARM_COLOR);
    this.setTooltip("Trả về giá trị S2 hiện tại");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_get_S2'] = function(block) {
  unoarmUse();
  return ['kdi_unoarm.S2_now', Blockly.Python.ORDER_ATOMIC];
};

// --- Block: Get current h_now (Number) ---
Blockly.Blocks['unoarm_get_S3'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Giá trị S3 hiện tại");
    this.setOutput(true, "Number");
    this.setColour(ARM_COLOR);
    this.setTooltip("Trả về giá trị S3 hiện tại");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_get_S3'] = function(block) {
  unoarmUse();
  return ['kdi_unoarm.S3_now', Blockly.Python.ORDER_ATOMIC];
};


// --- Block: Xoay đế S1 ---
Blockly.Blocks['unoarm_rotate_s1'] = {
  init: function() {
    this.appendValueInput("ANGLE")
        .setCheck("Number")
        .appendField("Xoay đế S1 đến góc:");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Xoay đế S1 từ 0 đến 180 độ (90 là chính giữa)");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_rotate_s1'] = function(block) {
  unoarmUse();
  var a = Blockly.Python.valueToCode(block, 'ANGLE', Blockly.Python.ORDER_ATOMIC) || '90';
  return 'await kdi_unoarm.quay_ve_goc(' + a + ')\n';
};

// --- Block: Điều khiển đầu hút (M1) ---
Blockly.Blocks['unoarm_control_sucker'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Đầu hút (M1):")
        .appendField(new Blockly.FieldDropdown([
          ["bật (hút)", "1"],
          ["tắt (thả)", "0"]
        ]), "ACTION");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Bật hoặc tắt đầu hút chân không cắm tại cổng động cơ M1");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_control_sucker'] = function(block) {
  unoarmUse();
  var act = block.getFieldValue('ACTION');
  return 'await kdi_unoarm.dieu_khien_dau_hut(' + (act === '1' ? 'True' : 'False') + ')\n';
};

// --- Block: Đặt tốc độ (%) ---
Blockly.Blocks['unoarm_set_speed'] = {
  init: function() {
    this.appendValueInput("SPEED")
        .setCheck("Number")
        .appendField("Đặt tốc độ cánh tay (%)");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Tốc độ từ 1 đến 100 %");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_set_speed'] = function(block) {
  unoarmUse();
  var s = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_ATOMIC) || '50';
  return 'kdi_unoarm.dat_toc_do_phan_tram(' + s + ')\n';
};

// --- Block: Về vị trí đầu ---
Blockly.Blocks['unoarm_go_home'] = {
  init: function() {
    this.appendDummyInput().appendField("Về vị trí đầu (S2 = 10, S3 = 10)");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Đưa cánh tay về tư thế bắt đầu an toàn");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_go_home'] = function(block) {
  unoarmUse();
  return 'await kdi_unoarm.ve_vi_tri_dau()\n';
};

// --- Block: Dừng lệnh đang chạy ---
Blockly.Blocks['unoarm_stop'] = {
  init: function() {
    this.appendDummyInput().appendField("Dừng cánh tay ngay");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Dừng ngay lệnh di chuyển đang chạy");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_stop'] = function(block) {
  unoarmUse();
  return 'kdi_unoarm.dung_lai()\n';
};


// --- Block: Giới hạn an toàn (0 = không giới hạn) ---
Blockly.Blocks['unoarm_set_limits'] = {
  init: function() {
    this.appendDummyInput().appendField("Giới hạn an toàn: tối đa mỗi lần (độ)");
    this.appendValueInput("MAXSTEP").setCheck("Number");
    this.appendDummyInput().appendField("h tối thiểu (cm)");
    this.appendValueInput("HMIN").setCheck("Number");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(ARM_COLOR);
    this.setTooltip("Nhập 0 để bỏ giới hạn tương ứng");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_set_limits'] = function(block) {
  unoarmUse();
  var m = Blockly.Python.valueToCode(block, 'MAXSTEP', Blockly.Python.ORDER_ATOMIC) || '0';
  var h = Blockly.Python.valueToCode(block, 'HMIN', Blockly.Python.ORDER_ATOMIC) || '0';
  return 'kdi_unoarm.dat_gioi_han((' + m + ') or None, (' + h + ') or None)\n';
};

// --- Block: Có thể tới (r, h)? (Boolean) ---
Blockly.Blocks['unoarm_can_reach'] = {
  init: function() {
    this.appendDummyInput().appendField("Có thể tới r:");
    this.appendValueInput("R").setCheck("Number");
    this.appendDummyInput().appendField("h:");
    this.appendValueInput("H").setCheck("Number");
    this.setInputsInline(true);
    this.setOutput(true, "Boolean");
    this.setColour(ARM_COLOR);
    this.setTooltip("True nếu từ vị trí hiện tại đi thẳng tới được (r, h)");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_can_reach'] = function(block) {
  unoarmUse();
  var r = Blockly.Python.valueToCode(block, 'R', Blockly.Python.ORDER_ATOMIC) || '0';
  var h = Blockly.Python.valueToCode(block, 'H', Blockly.Python.ORDER_ATOMIC) || '0';
  return ['kdi_unoarm.co_the_toi(' + r + ', ' + h + ')', Blockly.Python.ORDER_FUNCTION_CALL];
};

// --- Block: Giá trị S1 hiện tại ---
Blockly.Blocks['unoarm_get_S1'] = {
  init: function() {
    this.appendDummyInput().appendField("Giá trị S1 hiện tại");
    this.setOutput(true, "Number");
    this.setColour(ARM_COLOR);
    this.setTooltip("Trả về góc S1 (xoay đế) hiện tại");
    this.setHelpUrl("");
  }
};
Blockly.Python['unoarm_get_S1'] = function(block) {
  unoarmUse();
  return ['kdi_unoarm.S1_now', Blockly.Python.ORDER_ATOMIC];
};
