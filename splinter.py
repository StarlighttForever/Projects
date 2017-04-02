
from splinter import Browser 
from time import sleep
       
with Browser() as browser: 
     # Visit URL 

     url = "http://www.studentservices.auckland.ac.nz/en/admin/promo-buttons/sign-into-student-services-online.html" 
     browser.visit(url) 
     browser.fill('j_username', '650773285')
     browser.fill('j_password', 'jfirjfir')
     # Find and click the 'search' button 
     browser.click_link_by_href('#')
     # Interact with elements



     #enrol_button = browser.find_by_css('SSS_STYLESHEET_2.CSS:117')
     #enrol_button.click()
     #javascript:submitAction_win0(document.win0,'DERIVED_SSS_SCR_SSS_LINK_ANCHOR1');
     #javascript:submitAction_win0(document.win0,'DERIVED_SSS_SCL_SSS_ENRL_CART$56$');
     #list = browser.find_by_name('DERIVED_SSS_SCL_SSS_ENRL_CART$56$')
     #list.mouse_over().click()
     #browser.find_by_id('win0divDERIVED_SSS_SCL_SSS_ENRL_CART').first.click()
     #browser.execute_script("javascript:submitAction_win0(document.win0,'DERIVED_SSS_SCR_SSS_LINK_ANCHOR1');")
     browser.execute_script("javascript: submitAction_win0(document.win0,'CLASS_SRCH_WRK2_SSR_PB_CLASS_SRCH');")

     sleep(99999999)
     if browser.is_text_present('splinter.cobrateam.info'): 
         print "Yes, the official website was found!" 
     else:
         print "No, it wasn't found... We need to improve our SEO techniques"
     