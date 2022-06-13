package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;

/**
 * Interface used for geometry objects with reference frames to provide basic comparison methods for
 * convenience.
 * 
 * @author joh
 */
public interface EuclidFrameGeometry extends EuclidGeometry, ReferenceFrameHolder
{
   /**
    * Tests if {@code this} is equal to {@code other} to an {@code epsilon}. The test is usually
    * achieved on a per component basis. Sometimes a failing test does not necessarily mean that the
    * two objects are different in a geometric way. The method checks if the frame matches. If the
    * frames match, it returns {@code true}. Otherwise, redirects to
    * {@link #epsilonEquals(EuclidGeometry, double)}.
    *
    * @param frameGeometry the frameGeometry to compare against this. Not modified.
    * @param epsilon       tolerance to use when comparing each component.
    * @return {@code true} if the two objects are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(EuclidFrameGeometry frameGeometry, double epsilon)
   {
      if (getReferenceFrame() != frameGeometry.getReferenceFrame())
         return false;
      return epsilonEquals((EuclidGeometry) frameGeometry, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same geometry to an {@code epsilon}.
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will be
    * considered geometrically equal if they are at a distance from each other that is less or equal
    * than {@code epsilon}. The method checks if the frame matches. If the frames match, it returns
    * {@code true}. Otherwise, redirects to {@link #geometricallyEquals(EuclidGeometry, double)}.
    * </p>
    *
    * @param frameGeometry the frameGeometry to compare against this. Not modified.
    * @param epsilon       tolerance to use when comparing the two objects, usually refers to a
    *                      distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(EuclidFrameGeometry frameGeometry, double epsilon)
   {
      if (getReferenceFrame() != frameGeometry.getReferenceFrame())
         return false;
      return geometricallyEquals((EuclidGeometry) frameGeometry, epsilon);
   }

   /**
    * Tests if {@code this} is equal to {@code other}. The test is usually achieved on a per component
    * basis. Sometimes a failing test does not necessarily mean that the two objects are different in a
    * geometric way. The method checks if the frame matches. If the frames match, it returns
    * {@code true}. Otherwise, redirects to {@link #equals(Object)}.
    *
    * @param frameGeometry the frameGeometry to compare against this. Not modified.
    * @param epsilon       tolerance to use when comparing each component.
    * @return {@code true} if the two objects are equal component-wise, {@code false} otherwise.
    */
   default boolean equals(EuclidFrameGeometry frameGeometry)
   {
      if (getReferenceFrame() != frameGeometry.getReferenceFrame())
         return false;
      return equals((Object) frameGeometry);
   }
}
