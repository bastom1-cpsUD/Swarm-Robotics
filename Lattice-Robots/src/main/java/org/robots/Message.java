package org.robots;

public class Message {
    //Placeholder for message content, currently only includes authority list for leader election

    private AuthorityList authority;
    public Message(AuthorityList authority) {
        this.authority = authority;
    }
    public AuthorityList getAuthority() {
        return authority;
    }
}
