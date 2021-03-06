Components.utils.import("resource://gre/modules/Services.jsm");
Components.utils.import("resource://gre/modules/XPCOMUtils.jsm");

function createURI(s) {
  let service = Components.classes["@mozilla.org/network/io-service;1"]
                .getService(Components.interfaces.nsIIOService);
  return service.newURI(s, null, null);
}
 
function run_test() {
  // Set up a profile.
  do_get_profile();

  var secMan = Cc["@mozilla.org/scriptsecuritymanager;1"].getService(Ci.nsIScriptSecurityManager);
  const kURI1 = "http://example.com";
  var app1 = secMan.getAppCodebasePrincipal(createURI(kURI1), 1, false);
  var app10 = secMan.getAppCodebasePrincipal(createURI(kURI1), 10, false);
  var app1browser = secMan.getAppCodebasePrincipal(createURI(kURI1), 1, true);

  var am = Cc["@mozilla.org/network/http-auth-manager;1"].
           getService(Ci.nsIHttpAuthManager);
  am.setAuthIdentity("http", "a.example.com", -1, "basic", "realm", "", "example.com", "user", "pass", false, app1);
  am.setAuthIdentity("http", "a.example.com", -1, "basic", "realm", "", "example.com", "user3", "pass3", false, app1browser);
  am.setAuthIdentity("http", "a.example.com", -1, "basic", "realm", "", "example.com", "user2", "pass2", false, app10);

  let subject = {
    appId: 1,
    browserOnly: true,
    QueryInterface: XPCOMUtils.generateQI([Ci.mozIApplicationClearPrivateDataParams])
  };
  Services.obs.notifyObservers(subject, "webapps-clear-data", null);
  
  var domain = {value: ""}, user = {value: ""}, pass = {value: ""};
  try {
    am.getAuthIdentity("http", "a.example.com", -1, "basic", "realm", "", domain, user, pass, false, app1browser);
    do_check_false(true); // no identity should be present
  } catch (x) {
    do_check_eq(domain.value, "");
    do_check_eq(user.value, "");
    do_check_eq(pass.value, "");
  }

  am.getAuthIdentity("http", "a.example.com", -1, "basic", "realm", "", domain, user, pass, false, app1);
  do_check_eq(domain.value, "example.com");
  do_check_eq(user.value, "user");
  do_check_eq(pass.value, "pass");


  am.getAuthIdentity("http", "a.example.com", -1, "basic", "realm", "", domain, user, pass, false, app10);
  do_check_eq(domain.value, "example.com");
  do_check_eq(user.value, "user2");
  do_check_eq(pass.value, "pass2");
}
