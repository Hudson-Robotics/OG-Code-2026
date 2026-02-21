// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Enum representing all AprilTag locations on the 2026 FRC field.
 *
 * Coordinates are in inches. The origin is the top-right corner of the field
 * (as viewed from above with Blue Alliance on the left).
 * +X is toward the Red Alliance Station, +Y is away from the Scoring Table,
 * +Z is up from the carpet. Z-rotation: 0° faces Red, 90° faces audience,
 * 180° faces Blue.
 *
 * Source: AndyMark Perimeter field layout spec.
 */
public enum AprilTagLocation {

  // --- Red Alliance ---
  RED_TRENCH_1       ( 1,  "Trench",  Alliance.RED,  467.08, 291.79, 35.00, 180),
  RED_HUB_2          ( 2,  "Hub",     Alliance.RED,  468.56, 182.08, 44.25,  90),
  RED_HUB_3          ( 3,  "Hub",     Alliance.RED,  444.80, 172.32, 44.25, 180),
  RED_HUB_4          ( 4,  "Hub",     Alliance.RED,  444.80, 158.32, 44.25, 180),
  RED_HUB_5          ( 5,  "Hub",     Alliance.RED,  468.56, 134.56, 44.25, 270),
  RED_TRENCH_6       ( 6,  "Trench",  Alliance.RED,  467.08,  24.85, 35.00, 180),
  RED_TRENCH_7       ( 7,  "Trench",  Alliance.RED,  470.03,  24.85, 35.00,   0),
  RED_HUB_8          ( 8,  "Hub",     Alliance.RED,  482.56, 134.56, 44.25, 270),
  RED_HUB_9          ( 9,  "Hub",     Alliance.RED,  492.33, 144.32, 44.25,   0),
  RED_HUB_10         (10,  "Hub",     Alliance.RED,  492.33, 158.32, 44.25,   0),
  RED_HUB_11         (11,  "Hub",     Alliance.RED,  482.56, 182.08, 44.25,  90),
  RED_TRENCH_12      (12,  "Trench",  Alliance.RED,  470.03, 291.79, 35.00,   0),
  RED_OUTPOST_13     (13,  "Outpost", Alliance.RED,  649.58, 291.02, 21.75, 180),
  RED_OUTPOST_14     (14,  "Outpost", Alliance.RED,  649.58, 274.02, 21.75, 180),
  RED_TOWER_15       (15,  "Tower",   Alliance.RED,  649.57, 169.78, 21.75, 180),
  RED_TOWER_16       (16,  "Tower",   Alliance.RED,  649.57, 152.78, 21.75, 180),

  // --- Blue Alliance ---
  BLUE_TRENCH_17     (17,  "Trench",  Alliance.BLUE, 183.03,  24.85, 35.00,   0),
  BLUE_HUB_18        (18,  "Hub",     Alliance.BLUE, 181.56, 134.56, 44.25, 270),
  BLUE_HUB_19        (19,  "Hub",     Alliance.BLUE, 205.32, 144.32, 44.25,   0),
  BLUE_HUB_20        (20,  "Hub",     Alliance.BLUE, 205.32, 158.32, 44.25,   0),
  BLUE_HUB_21        (21,  "Hub",     Alliance.BLUE, 181.56, 182.08, 44.25,  90),
  BLUE_TRENCH_22     (22,  "Trench",  Alliance.BLUE, 183.03, 291.79, 35.00,   0),
  BLUE_TRENCH_23     (23,  "Trench",  Alliance.BLUE, 180.08, 291.79, 35.00, 180),
  BLUE_HUB_24        (24,  "Hub",     Alliance.BLUE, 167.56, 182.08, 44.25,  90),
  BLUE_HUB_25        (25,  "Hub",     Alliance.BLUE, 157.79, 172.32, 44.25, 180),
  BLUE_HUB_26        (26,  "Hub",     Alliance.BLUE, 157.79, 158.32, 44.25, 180),
  BLUE_HUB_27        (27,  "Hub",     Alliance.BLUE, 167.56, 134.56, 44.25, 270),
  BLUE_TRENCH_28     (28,  "Trench",  Alliance.BLUE, 180.08,  24.85, 35.00, 180),
  BLUE_OUTPOST_29    (29,  "Outpost", Alliance.BLUE,   0.54,  25.62, 21.75,   0),
  BLUE_OUTPOST_30    (30,  "Outpost", Alliance.BLUE,   0.54,  42.62, 21.75,   0),
  BLUE_TOWER_31      (31,  "Tower",   Alliance.BLUE,   0.55, 146.86, 21.75,   0),
  BLUE_TOWER_32      (32,  "Tower",   Alliance.BLUE,   0.55, 163.86, 21.75,   0);

  // -------------------------------------------------------------------------

  public enum Alliance { RED, BLUE }

  public final int id;
  public final String fieldElement;
  public final Alliance alliance;
  /** X coordinate in inches (+X toward Red Alliance Station) */
  public final double x;
  /** Y coordinate in inches (+Y away from Scoring Table) */
  public final double y;
  /** Z coordinate in inches (height from carpet) */
  public final double z;
  /** Z-rotation in degrees (0°=faces Red, 90°=faces audience, 180°=faces Blue) */
  public final double rotation;

  AprilTagLocation(int id, String fieldElement, Alliance alliance,
      double x, double y, double z, double rotation) {
    this.id = id;
    this.fieldElement = fieldElement;
    this.alliance = alliance;
    this.x = x;
    this.y = y;
    this.z = z;
    this.rotation = rotation;
  }

  /**
   * Returns the AprilTagLocation for the given tag ID, or null if not found.
   *
   * @param id The AprilTag ID reported by the Limelight (e.g. from getTID())
   * @return The matching AprilTagLocation, or null
   */
  public static AprilTagLocation fromId(int id) {
    for (AprilTagLocation tag : values()) {
      if (tag.id == id) return tag;
    }
    return null;
  }

  @Override
  public String toString() {
    return String.format("Tag %d (%s %s) @ (%.2f, %.2f, %.2f) facing %.0f°",
        id, alliance, fieldElement, x, y, z, rotation);
  }
}
