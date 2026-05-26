package org.communicationModels;

import org.graphs.LatticeEdge;

public class Message {
    //Placeholder for message content, currently only includes authority list for leader election

    private AuthorityList authority;
    LatticeEdge assignment;

    public Message(AuthorityList authority, LatticeEdge assignment) {
        this.authority = authority;
        this.assignment = assignment;
    }
    public AuthorityList getAuthority() {
        return authority;
    }

    public LatticeEdge getAssignment() {
        return assignment;
    }
}
