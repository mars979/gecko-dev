/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

package org.mozilla.gecko.sync.repositories;

import org.mozilla.gecko.sync.SyncException;

public class MultipleRecordsForGuidException extends SyncException {

  private static final long serialVersionUID = 7426987323485324741L;

  public MultipleRecordsForGuidException(Exception ex) {
    super(ex);
  }
}
