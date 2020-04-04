package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Factory for creating a specific type of face.
 *
 * @author Sylvain Bertrand
 * @param <Face> the final type used for representing a face.
 */
@FunctionalInterface
public interface Face3DFactory<Face extends Face3DReadOnly>
{
   /**
    * Creates and initializes a new face.
    *
    * @param initialNormalGuess  a guess for what the normal of the new face should be. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    * @return the new face.
    */
   Face newInstance(Vector3DReadOnly initialNormalGuess, double constructionEpsilon);
}
