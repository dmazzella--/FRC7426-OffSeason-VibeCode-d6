// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/**
 * ReefState is a singleton that maintains the state of a reef.
 * <p>
 * It contains an array of ReefSide objects, each representing a side of the reef.
 * Each ReefSide contains two Branch objects, which track the state of coral
 * on three levels (L2, L3, and L4).
 * </p>
 */
public class ReefState {
    private static ReefState m_instance;
  
    /**
     * Returns the singleton instance of ReefState.
     * <p>
     * If no instance exists, a new one is created.
     * </p>
     *
     * @return the singleton instance of ReefState.
     */
    public static ReefState getInstance() {
        if (m_instance == null) {
            m_instance = new ReefState();
        }
        return m_instance;
    }

    /**
     * Represents a side of the reef. Each side consists of two branches.
     */
    public static class ReefSide {
        
        /**
         * Represents a branch on a reef side.
         * <p>
         * Each branch tracks the state of coral at three levels: L2, L3, and L4.
         * The levels are stored as booleans in an array, with {@code false} indicating
         * an inactive state.
         * </p>
         */
        public static class Branch {
            private boolean[] m_coral;
            private String m_name;

            /**
             * Constructs a new Branch with the given name.
             * <p>
             * Initializes three levels (L2, L3, and L4) to {@code false}.
             * </p>
             *
             * @param name the name of the branch.
             */
            public Branch(String name) {
                // {L2, L3, L4}
                m_coral = new boolean[]{false, false, false};
                m_name = name;
            }

            /**
             * Returns the name of the branch.
             *
             * @return the branch name.
             */
            public String getName() {
                return m_name;
            }

            /**
             * Returns the state of the specified level.
             * <p>
             * The {@code level} parameter corresponds to one of L2, L3, or L4.
             * It is translated to an array index (level - 2).
             * </p>
             *
             * @param level the level to check (should be L2, L3, or L4).
             * @return {@code true} if the level is active, {@code false} otherwise.
             * @throws IllegalArgumentException if the level is not L2, L3, or L4.
             */
            public boolean getLevel(int level) {
                level = level - 2; // Translate level into corresponding array index

                // Verify that only L2, L3, and L4 are usable
                if (level < 0 || level > 2) {
                    throw new IllegalArgumentException("Invalid level. Available levels are L2, L3, and L4.");
                }

                // Return score state of level
                return m_coral[level];
            }

            /**
             * Sets the state of the specified level.
             * <p>
             * The level is specified as L2, L3, or L4 and is translated to an array index.
             * </p>
             *
             * @param level the level to set (should be L2, L3, or L4).
             * @param state the state to set for the level.
             * @throws IllegalArgumentException if the level is not L2, L3, or L4.
             */
            public void setLevel(int level, boolean state) {
                level = level - 2; // Translate level into corresponding array index

                // Verify that only L2, L3, and L4 are usable
                if (level < 0 || level > 2) {
                    throw new IllegalArgumentException("Invalid level. Available levels are L2, L3, and L4.");
                }

                // Set state
                m_coral[level] = state;
            }

            /**
             * Toggles the state of the specified level.
             * <p>
             * The current state of the level is inverted.
             * </p>
             *
             * @param level the level to toggle (should be L2, L3, or L4).
             * @throws IllegalArgumentException if the level is not L2, L3, or L4.
             */
            public void toggleLevel(int level) {
                // Get current state
                boolean currentState = getLevel(level);

                // Invert level state
                setLevel(level, !currentState);
            }
        }

        private Branch[] m_branches;

        /**
         * Constructs a new ReefSide with two branches.
         *
         * @param leftBranchName  the name for the left branch.
         * @param rightBranchName the name for the right branch.
         */
        public ReefSide(String leftBranchName, String rightBranchName) {
            m_branches = new Branch[]{
                new Branch(leftBranchName),
                new Branch(rightBranchName)
            };
        }

        /**
         * Returns the branch at the specified index.
         *
         * @param branch the index of the branch (0 for left branch, 1 for right branch).
         * @return the {@link Branch} at the specified index.
         * @throws IllegalArgumentException if the branch index is invalid.
         */
        public Branch getBranch(int branch) {
            if (branch < 0 || branch > 1) {
                throw new IllegalArgumentException("Invalid branch. Available branches are 0 (Left Branch) and 1 (Right Branch).");
            }

            return m_branches[branch];
        }

        /**
         * Represents a coordinate within a reef side's branch structure.
         * <p>
         * A coordinate is defined by a branch index and a level number.
         * The branch index is 0 for the left branch and 1 for the right branch.
         * The level number corresponds to L2, L3, or L4.
         * </p>
         *
         * @param branch the index of the branch.
         * @param level  the level number (2 for L2, 3 for L3, 4 for L4).
         */
        public record CoralCoordinate(int branch, int level) {}

        /**
         * Returns the coordinate of the best available (empty) coral spot on this reef side.
         * <p>
         * The search is performed in the following order:
         * <ol>
         *   <li>For level L4 (top), check left branch (branch 0) then right branch (branch 1).</li>
         *   <li>For level L3, check left branch then right branch.</li>
         *   <li>For level L2 (bottom), check left branch then right branch.</li>
         * </ol>
         * The first coordinate found where the coral state is not scored (i.e., {@code false})
         * is returned.
         * </p>
         *
         * @return the {@link CoralCoordinate} representing the first empty spot,
         *         or {@code null} if all spots are scored.
         */
        public CoralCoordinate getBestEmptyCoordinate() {
            // Iterate over levels from L4 (top) down to L2 (bottom)
            for (int level = 4; level >= 2; level--) {
                // For each level, check branches left to right (0 then 1)
                for (int branchIndex = 0; branchIndex < m_branches.length; branchIndex++) {
                    if (!m_branches[branchIndex].getLevel(level)) {
                        return new CoralCoordinate(branchIndex, level);
                    }
                }
            }
            // Return null if no empty spot is found
            return null;
        }
    }

    private ReefSide[] m_sides;

    /**
     * Private constructor for the singleton ReefState.
     * <p>
     * Initializes six ReefSide objects with predefined branch names.
     * </p>
     */
    private ReefState() {
        m_sides = new ReefSide[]{
            new ReefSide("A", "B"),
            new ReefSide("C", "D"),
            new ReefSide("E", "F"),
            new ReefSide("G", "H"),
            new ReefSide("I", "J"),
            new ReefSide("K", "L"),
        };
    }

    /**
     * Sets the state of a specified level in the reef.
     *
     * @param side   the index of the reef side.
     * @param branch the index of the branch within the side.
     * @param level  the level (L2, L3, or L4) to set.
     * @param state  the state to set for the specified level.
     */
    public void setLevel(int side, int branch, int level, boolean state) {
        m_sides[side].getBranch(branch).setLevel(level, state);
    }

    /**
     * Returns the state of a specified level in the reef.
     *
     * @param side   the index of the reef side.
     * @param branch the index of the branch within the side.
     * @param level  the level (L2, L3, or L4) to check.
     * @return {@code true} if the level is active, {@code false} otherwise.
     */
    public boolean getLevel(int side, int branch, int level) {
        return m_sides[side].getBranch(branch).getLevel(level);
    }

    /**
     * Toggles the state of a specified level in the reef.
     * <p>
     * The current state of the level is inverted.
     * </p>
     *
     * @param side   the index of the reef side.
     * @param branch the index of the branch within the side.
     * @param level  the level (L2, L3, or L4) to toggle.
     */
    public void toggleLevel(int side, int branch, int level) {
        m_sides[side].getBranch(branch).toggleLevel(level);
    }
}
