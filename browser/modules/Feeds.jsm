/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

this.EXPORTED_SYMBOLS = [ "Feeds" ];

Components.utils.import("resource://gre/modules/XPCOMUtils.jsm");
Components.utils.import("resource://gre/modules/Services.jsm");

XPCOMUtils.defineLazyModuleGetter(this, "BrowserUtils",
                                  "resource://gre/modules/BrowserUtils.jsm");

const { interfaces: Ci, classes: Cc } = Components;

this.Feeds = {
  init() {
    let mm = Cc["@mozilla.org/globalmessagemanager;1"].getService(Ci.nsIMessageListenerManager);
    mm.addMessageListener("WCCR:registerProtocolHandler", this);
  },

  receiveMessage(aMessage) {
    switch (aMessage.name) {
      case "WCCR:registerProtocolHandler": {
        let data = aMessage.data;
        let registrar = Cc["@mozilla.org/embeddor.implemented/web-content-handler-registrar;1"].
                          getService(Ci.nsIWebContentHandlerRegistrar);
        registrar.registerProtocolHandler(data.protocol, data.uri, data.title,
                                          aMessage.target);
        break;
      }

      case "WCCR:registerContentHandler": {
        let data = aMessage.data;
        let registrar = Cc["@mozilla.org/embeddor.implemented/web-content-handler-registrar;1"].
                          getService(Ci.nsIWebContentHandlerRegistrar);
        registrar.registerContentHandler(data.contentType, data.uri, data.title,
                                         aMessage.target);
        break;
      }
    }
  },

  /**
   * isValidFeed: checks whether the given data represents a valid feed.
   *
   * @param  aLink
   *         An object representing a feed with title, href and type.
   * @param  aPrincipal
   *         The principal of the document, used for security check.
   * @param  aIsFeed
   *         Whether this is already a known feed or not, if true only a security
   *         check will be performed.
   */
  isValidFeed: function(aLink, aPrincipal, aIsFeed) {
    if (!aLink || !aPrincipal)
      return false;

    var type = aLink.type.toLowerCase().replace(/^\s+|\s*(?:;.*)?$/g, "");
    if (!aIsFeed) {
      aIsFeed = (type == "application/rss+xml" ||
                 type == "application/atom+xml");
    }

    if (aIsFeed) {
      // re-create the principal as it may be a CPOW.
      let principalURI = BrowserUtils.makeURIFromCPOW(aPrincipal.URI);
      let principalToCheck = Services.scriptSecurityManager.getNoAppCodebasePrincipal(principalURI);
      try {
        BrowserUtils.urlSecurityCheck(aLink.href, principalToCheck,
                                      Ci.nsIScriptSecurityManager.DISALLOW_INHERIT_PRINCIPAL);
        return type || "application/rss+xml";
      }
      catch(ex) {
      }
    }

    return null;
  },

};
