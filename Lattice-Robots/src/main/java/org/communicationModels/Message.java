package org.communicationModels;

import java.util.PriorityQueue;

import org.graphs.LatticeEdge;

/**
 * A class representing a message that can be sent between robots in the communication system.
 */
public class Message implements Comparable<Message> {
    /**
     * The authority list associated with the sender robot
     */
    private AuthorityList authority;

    /**
     * The lattice edge assignment for the recieving robot, which may be null if the message is not an assignment message
     */
    private LatticeEdge assignment;

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

    public int compareTo(Message other) {
        return authority.compareTo(other.authority);
    }
}
