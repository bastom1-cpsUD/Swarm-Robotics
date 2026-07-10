package org.communicationModels.SongOkaneComms;
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

    /**
     * Checks is the authorityId provided is the root of the authority list
     * @param authorityId
     * @return  true if the first authority in the list is the root, false otherwise
     */
    public boolean isRoot(int authorityId) {
        return getRootAuthority() == authorityId;
    }
    /**
     * Compares this AuthorityList with another AuthorityList for ordering.
     * @param other the other AuthorityList to compare with
     * @return a negative integer, zero, or a positive integer as this object is less than, equal to, or greater than the specified object
     */
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

    /**
     * Compares this AuthorityList with another AuthorityList for ordering based on the following criteria:
     * 1. The AuthorityList with the smaller root authority ID is considered higher.
     * 2. If both AuthorityLists have the same root authority ID, the one with fewer total authorities is considered higher.
     * 3. If both AuthorityLists have the same root authority ID and the same number of authorities, the one with the smaller most recently added authority ID is considered higher.
     * If all criteria are the same, the AuthorityLists are considered equal.
     * @param other the other AuthorityList to compare with
     * @return a negative integer, zero, or a positive integer as this object is less
     */
    @Override
    public int compareTo(AuthorityList other) {
        //Case 1: A1 has higher authority than A2 if A1 root is greater than A2 root
        if(this.authorities.get(0) < other.authorities.get(0)) {
            return -1;   
        } else if(this.authorities.get(0) > other.authorities.get(0)) {
            return 1;
        }
        //Case 2: If A1 and A2 have the same root, but A1 has fewer authorities than A2, then A1 is higher than A2
        else if(this.authorities.size() < other.authorities.size()) {
            return -1;
        } else if(this.authorities.size() > other.authorities.size()) {
            return 1;

        //Case 3: If A1 and A2 have the same root and the same number of authorities, but A1's most recently added authority is greater than A2'
        } else if(this.authorities.get(this.authorities.size() - 1) < other.authorities.get(other.authorities.size() - 1)) {
            return -1;
        } else if(this.authorities.get(this.authorities.size() - 1) > other.authorities.get(other.authorities.size() - 1)) {
            return 1;
        }
        return 0;
    }

}

