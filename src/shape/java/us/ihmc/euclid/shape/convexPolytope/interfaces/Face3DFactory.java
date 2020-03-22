package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Face3DFactory<Face extends Face3DReadOnly>
{
   Face newInstance(Vector3DReadOnly initialNormalGuess, double constructionEpsilon);
}
