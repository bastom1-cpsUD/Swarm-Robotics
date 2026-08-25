package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.ListIterator;

import org.graphs.voltage.Face;
import org.graphs.voltage.HalfEdge;

public class FaceObligationSet {
    private LinkedHashMap<Integer, FaceObligation> obligationSet;

    public FaceObligationSet() {
        this.obligationSet = new LinkedHashMap<>();
    }

    public FaceObligation getOrCreateObligation(int parentId, int edgeId) {
        if(obligationSet.containsKey(edgeId)) return obligationSet.get(edgeId);
        
        FaceObligation obligation = new FaceObligation(parentId, edgeId);
        this.obligationSet.put(edgeId, obligation);
        return obligation;
    }

    public FaceObligation findByEdge(HalfEdge edge) {
        return obligationSet.get(edge.getId());
    }

    public FaceObligation findUnfulfilled() {
        List<FaceObligation> obligations = new ArrayList<>(obligationSet.values());

        ListIterator<FaceObligation> iter = obligations.listIterator();

        while(iter.hasNext()) {
            FaceObligation cur = iter.next();
            if(cur.isUnfilled()) return cur;
        }
        return null;
    }

    public FaceObligation findByChild(int childId) {
        List<FaceObligation> obligations = new ArrayList<>(obligationSet.values());

        ListIterator<FaceObligation> iter = obligations.listIterator();

        while(iter.hasNext()) {
            FaceObligation cur = iter.next();
            if(cur.matchesChild(childId)) return cur;
        }
        return null;
    }

    public boolean remove(FaceObligation obligation) {
        FaceObligation other = obligationSet.get(obligation.getEdgeId());
        if(other == null) {
            return false;
        }

        if(other.equals(obligation)) {
            obligationSet.remove(obligation.getEdgeId());
            return true;
        }
        return false;
    }

    public void clearAll() {
        obligationSet.clear();
    }

}
