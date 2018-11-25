package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytopeFaceProvider;

public class ConvexPolytopeFaceBuilder implements ConvexPolytopeFaceProvider
{

   @Override
   public Face3D getFace()
   {
      return new Face3D();
   }
}
