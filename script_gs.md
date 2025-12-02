/**
 * Google Apps Script Web App - ESP32 Data Logger Endpoint
 * 
 * DEPLOYMENT INSTRUCTIONS:
 * 1. Create a new Google Sheet for your data
 * 2. Go to Extensions > Apps Script
 * 3. Paste this code into Code.gs
 * 4. Deploy as Web App (Deploy > New deployment)
 *    - Execute as: Me
 *    - Who has access: Anyone (or Anyone with Google account if you want auth)
 * 5. Copy the Web App URL
 * 6. Use that URL in your ESP32: sheet_url <URL>
 */
// Configuration
const SHEET_NAME = "SensorData"; // Name of sheet to write data to
const MAX_ROWS = 10000; // Maximum rows before creating new sheet
/**
 * Handle POST requests from ESP32
 */
function doPost(e) {
  try {
    // Parse JSON payload
    const data = JSON.parse(e.postData.contents);
    
    // Get or create sheet
    const sheet = getOrCreateSheet();
    
    // Check if this is a test request
    if (data.test === true) {
      return ContentService.createTextOutput(JSON.stringify({
        success: true,
        message: "Connection test successful!",
        timestamp: new Date().toISOString()
      })).setMimeType(ContentService.MimeType.JSON);
    }
    
    // Write data to sheet
    writeData(sheet, data);
    
    // Return success response
    return ContentService.createTextOutput(JSON.stringify({
      success: true,
      rowsWritten: Array.isArray(data.readings) ? data.readings.length : 1,
      timestamp: new Date().toISOString()
    })).setMimeType(ContentService.MimeType.JSON);
    
  } catch (error) {
    // Return error response
    return ContentService.createTextOutput(JSON.stringify({
      success: false,
      error: error.toString(),
      timestamp: new Date().toISOString()
    })).setMimeType(ContentService.MimeType.JSON);
  }
}
/**
 * Handle GET requests (for testing in browser)
 */
function doGet(e) {
  const html = `
    <html>
      <head>
        <title>ESP32 Data Logger Endpoint</title>
        <style>
          body { font-family: Arial; margin: 40px; }
          .status { padding: 20px; background: #e8f5e9; border-radius: 8px; }
          code { background: #f5f5f5; padding: 2px 6px; border-radius: 3px; }
        </style>
      </head>
      <body>
        <h1>ESP32 Data Logger Endpoint</h1>
        <div class="status">
          <h2>✅ Endpoint is Active</h2>
          <p>This endpoint is ready to receive data from your ESP32.</p>
          <p><strong>Sheet:</strong> ${SHEET_NAME}</p>
          <p><strong>Current URL:</strong><br><code>${ScriptApp.getService().getUrl()}</code></p>
        </div>
        
        <h2>Usage</h2>
        <p>Configure your ESP32 with:</p>
        <code>sheet_url ${ScriptApp.getService().getUrl()}</code>
        
        <h2>Test Connection</h2>
        <p>Send a POST request with JSON:</p>
        <pre>{ "test": true }</pre>
      </body>
    </html>
  `;
  
  return HtmlService.createHtmlOutput(html);
}
/**
 * Get or create the data sheet
 */
function getOrCreateSheet() {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  let sheet = ss.getSheetByName(SHEET_NAME);
  
  // Create sheet if it doesn't exist
  if (!sheet) {
    sheet = ss.insertSheet(SHEET_NAME);
    
    // Add headers
    sheet.getRange(1, 1, 1, 8).setValues([[
      "Timestamp",
      "Iteration",
      "Voltage (V)",
      "Current (A)",
      "Power (W)",
      "Energy Used (Wh)",
      "Energy Regen (Wh)",
      "RTC Time"
    ]]);
    
    // Format header row
    sheet.getRange(1, 1, 1, 8)
      .setFontWeight("bold")
      .setBackground("#4285f4")
      .setFontColor("#ffffff");
    
    // Freeze header row
    sheet.setFrozenRows(1);
  }
  
  // Check if sheet is getting too large
  if (sheet.getLastRow() > MAX_ROWS) {
    archiveSheet(sheet);
  }
  
  return sheet;
}
/**
 * Write data to sheet
 */
function writeData(sheet, data) {
  const timestamp = new Date();
  
  // Handle batch data (multiple readings)
  if (data.readings && Array.isArray(data.readings)) {
    const rows = data.readings.map(reading => [
      timestamp,
      reading.iteration || "",
      reading.voltage || 0,
      reading.current || 0,
      reading.power || 0,
      reading.wh_used || 0,
      reading.wh_regen || 0,
      reading.rtc_time || ""
    ]);
    
    sheet.getRange(sheet.getLastRow() + 1, 1, rows.length, 8).setValues(rows);
  } 
  // Handle single reading
  else {
    sheet.appendRow([
      timestamp,
      data.iteration || "",
      data.voltage || 0,
      data.current || 0,
      data.power || 0,
      data.wh_used || 0,
      data.wh_regen || 0,
      data.rtc_time || ""
    ]);
  }
}
/**
 * Archive old sheet when it gets too large
 */
function archiveSheet(sheet) {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const archiveName = SHEET_NAME + "_Archive_" + Utilities.formatDate(new Date(), "GMT", "yyyyMMdd_HHmmss");
  
  // Rename current sheet
  sheet.setName(archiveName);
  
  // Create new sheet with original name
  const newSheet = ss.insertSheet(SHEET_NAME, 0);
  
  // Copy header from archived sheet
  const headers = sheet.getRange(1, 1, 1, 8).getValues();
  newSheet.getRange(1, 1, 1, 8)
    .setValues(headers)
    .setFontWeight("bold")
    .setBackground("#4285f4")
    .setFontColor("#ffffff");
  
  newSheet.setFrozenRows(1);
}
/**
 * Utility: Clear all data (keep headers)
 * Run this manually if you want to reset the sheet
 */
function clearAllData() {
  const sheet = getOrCreateSheet();
  if (sheet.getLastRow() > 1) {
    sheet.deleteRows(2, sheet.getLastRow() - 1);
  }
  Logger.log("All data cleared from " + SHEET_NAME);
}
/**
 * Utility: Get statistics
 * Run this manually to see data stats
 */
function getStatistics() {
  const sheet = getOrCreateSheet();
  const lastRow = sheet.getLastRow();
  
  if (lastRow <= 1) {
    Logger.log("No data in sheet");
    return;
  }
  
  const data = sheet.getRange(2, 1, lastRow - 1, 8).getValues();
  
  Logger.log("Total Entries: " + data.length);
  Logger.log("First Entry: " + data[0][0]);
  Logger.log("Last Entry: " + data[data.length - 1][0]);
  
  // Calculate averages
  let sumVoltage = 0, sumCurrent = 0, sumPower = 0;
  data.forEach(row => {
    sumVoltage += row[2];
    sumCurrent += row[3];
    sumPower += row[4];
  });
  
  Logger.log("Average Voltage: " + (sumVoltage / data.length).toFixed(2) + " V");
  Logger.log("Average Current: " + (sumCurrent / data.length).toFixed(2) + " A");
  Logger.log("Average Power: " + (sumPower / data.length).toFixed(2) + " W");
}