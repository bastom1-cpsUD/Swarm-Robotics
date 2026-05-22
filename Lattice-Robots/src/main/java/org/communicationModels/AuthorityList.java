package org.communicationModels;
import java.util.ArrayList;
/**
 * A class to manage a list of authority IDs for a robot, with the first ID being the root authority, and the most
 * recently added authority being the last in the list. This class provides methods to add new authorities and retrieve the list of authorities.
 */
public class AuthorityList implements Comparable<AuthorityList> {
    private ArrayList<Integer> authorities;

    /**
     * Constructs an empty authority list.
     */
    public AuthorityList() {
        this.authorities = new ArrayList<>();
    }

    /**
     * Constructs an authority list with a single authority ID.
     *
     * @param authorityId the ID of the root authority
     */
    public AuthorityList(int authorityId) {
        this.authorities = new ArrayList<>();
        this.authorities.add(authorityId);
    }

    /**
     * Constructs an authority list with the given authority IDs.
     *
     * @param authorityIds the array of authority IDs
     */
    public AuthorityList(ArrayList<Integer> authorityIds) {
        this.authorities = authorityIds;
    }

    /**
     * Returns the list of authority IDs.
     * @return
     */
    public ArrayList<Integer> getAuthorities() {
        return new ArrayList<>(authorities);
    }

    /**
     * Adds a new authority ID to the list if it is not already present. The new authority is added to the end of the list.
     * @param authorityId
     */
    public void addAuthority(int authorityId) {
        if (authorities.contains(authorityId)) {
            return; // Authority already exists, do not add
        }
        authorities.add(authorityId);
    }

    /**
     * Returns the root authority ID.
     * @return the root authority ID
     */
    public int getRootAuthority() {
        if (authorities.isEmpty()) {
            throw new IllegalStateException("Authority list is empty. No root authority available.");
        }
        return authorities.get(0);
    }

    /**
     * Returns the most recently added authority ID.
     * @return the most recently added authority ID
     */
    public int getMostRecentAuthority() {
        if (authorities.isEmpty()) {
            throw new IllegalStateException("Authority list is empty. No authorities available.");
        }
        return authorities.get(authorities.size() - 1);
    }

    /**
     * Checks if the authority list contains a specific authority ID.
     * @param authorityId the ID of the authority to check
     * @return true if the authority is in the list, false otherwise
     */
    public boolean contains(int authorityId) {
        return authorities.contains(authorityId);
    }
    
    @Override
    public boolean equals(Object obj) {
        if(this == obj) return true;
        if(!(obj instanceof AuthorityList)) return false;
        AuthorityList other = (AuthorityList) obj;

        if(this.authorities.size() != other.authorities.size()) return false;
        for(int i = 0; i < this.authorities.size(); i++) {
            if(!this.authorities.get(i).equals(other.authorities.get(i))) {
                return false;
            }
        }
        return true;
    }

    @Override
    public int hashCode() {
        return authorities.hashCode();
    }

    @Override
    public int compareTo(AuthorityList other) {
        //Case 1: A1 has higher authority than A2 if A1 root is greater than A2 root
        if(this.authorities.get(0) > other.authorities.get(0)) {
            return -1;   
        } else if(this.authorities.get(0) < other.authorities.get(0)) {
            return 1;
        }
        //Case 2: If A1 and A2 have the same root, but A1 has fewer authorities than A2, then A1 is higher than A2
        else if(this.authorities.size() < other.authorities.size()) {
            return -1;
        } else if(this.authorities.size() > other.authorities.size()) {
            return 1;

        //Case 3: If A1 and A2 have the same root and the same number of authorities, but A1's most recently added authority is greater than A2'
        } else if(this.authorities.get(this.authorities.size() - 1) > other.authorities.get(other.authorities.size() - 1)) {
            return -1;
        } else if(this.authorities.get(this.authorities.size() - 1) < other.authorities.get(other.authorities.size() - 1)) {
            return 1;
        }
        return 0;
    }

}

