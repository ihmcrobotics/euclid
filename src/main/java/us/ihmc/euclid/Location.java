package us.ihmc.euclid;

/**
 * This enum aims to provide readable selection of relative positions between geometries.
 * <p>
 * These positions are not associated with the axes of the inertial coordinate system, i.e.
 * {@code AHEAD} is not associated with either x, y, or z directions.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public enum Location
{
   /** Indicates that an object is ahead or in front of another object. */
   AHEAD,
   /** Indicates that an object is behind of another object. */
   BEHIND,
   /** Indicates that an object is above or on top of another object. */
   ABOVE,
   /** Indicates that an object is below or under another object. */
   BELOW,
   /** Indicates that an object is to the left of another object. */
   LEFT,
   /** Indicates that an object is to the left of another object. */
   RIGHT,
   /** Indicates that an object is inside another object. */
   INSIDE,
   /** Indicates that an object is outside another object. */
   OUTSIDE;
}
