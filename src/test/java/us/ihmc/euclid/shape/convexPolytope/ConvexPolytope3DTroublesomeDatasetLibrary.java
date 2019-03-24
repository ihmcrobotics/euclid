package us.ihmc.euclid.shape.convexPolytope;

import java.util.Collections;
import java.util.Random;

import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tuple3D.Point3D;

public class ConvexPolytope3DTroublesomeDatasetLibrary
{
   public static class DatasetGJKNullPointerExceptionBug1Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug1Original()
      {
         pointsBeforeIssue.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
         pointsBeforeIssue.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
         pointsBeforeIssue.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
         pointsBeforeIssue.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
         pointsBeforeIssue.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
         pointsBeforeIssue.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
         pointsBeforeIssue.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
         pointsBeforeIssue.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
         troublesomePoint.set(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug1Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug1Simplified()
      {
         // pointsBeforeIssue.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
         // pointsBeforeIssue.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
         // pointsBeforeIssue.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
         pointsBeforeIssue.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
         pointsBeforeIssue.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
         pointsBeforeIssue.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
         // pointsBeforeIssue.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
         pointsBeforeIssue.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
         troublesomePoint.set(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2Original()
      {
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
         pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         pointsBeforeIssue.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
         troublesomePoint.set(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2OriginalV2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2OriginalV2()
      {
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         troublesomePoint.set(00.0269036070083187, 0.0083216605291816, 0.0078200543380451);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2OriginalV3 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2OriginalV3()
      {
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         troublesomePoint.set(0.0212072625303128, -0.0130016538284213, 0.0026586776633881);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2Simplified()
      {
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
         // pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         pointsBeforeIssue.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
         troublesomePoint.set(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3Original()
      {
         pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         pointsBeforeIssue.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
         pointsBeforeIssue.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
         pointsBeforeIssue.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
         pointsBeforeIssue.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
         pointsBeforeIssue.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
         pointsBeforeIssue.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
         troublesomePoint.set(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3OriginalV2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3OriginalV2()
      {
         pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         troublesomePoint.set(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         //      pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         pointsBeforeIssue.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
         pointsBeforeIssue.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
         //      pointsBeforeIssue.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
         //      pointsBeforeIssue.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
         //      pointsBeforeIssue.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
         pointsBeforeIssue.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
         troublesomePoint.set(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug4Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug4Original()
      {
         pointsBeforeIssue.add(new Point3D(0.0051506551364596, -0.0348819759911019, 0.0691778540151096));
         pointsBeforeIssue.add(new Point3D(0.0422493845451721, 0.0727920949601139, 0.5674466022012327));
         pointsBeforeIssue.add(new Point3D(0.0004104647457426, -0.0451992694194132, 0.0472529766744278));
         pointsBeforeIssue.add(new Point3D(0.0020291613573142, -0.0559465436341543, 0.0250833552689338));
         pointsBeforeIssue.add(new Point3D(-0.0033350301665630, -0.0940280104643934, -0.0447924048275297));
         pointsBeforeIssue.add(new Point3D(-0.0113077830899894, -0.1649473194674913, -0.1518735281452312));
         pointsBeforeIssue.add(new Point3D(-0.0253624237359933, -0.3387902241779525, -0.3406397827449963));
         pointsBeforeIssue.add(new Point3D(-0.0470958837642707, -1.2942558333752370, -0.6325393732333888));
         pointsBeforeIssue.add(new Point3D(0.0046484599886485, -0.0503176108042466, 0.0362947448605222));
         troublesomePoint.set(-0.0068611168376205, -0.0525067429256160, 0.0327580791751562);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug4Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug4Simplified()
      {
         pointsBeforeIssue.add(new Point3D(0.0051506551364596, -0.0348819759911019, 0.0691778540151096));
         // pointsBeforeIssue.add(new Point3D(0.0422493845451721, 0.0727920949601139, 0.5674466022012327));
         pointsBeforeIssue.add(new Point3D(0.0004104647457426, -0.0451992694194132, 0.0472529766744278));
         pointsBeforeIssue.add(new Point3D(0.0020291613573142, -0.0559465436341543, 0.0250833552689338));
         pointsBeforeIssue.add(new Point3D(-0.0033350301665630, -0.0940280104643934, -0.0447924048275297));
         // pointsBeforeIssue.add(new Point3D(-0.0113077830899894, -0.1649473194674913, -0.1518735281452312));
         // pointsBeforeIssue.add(new Point3D(-0.0253624237359933, -0.3387902241779525, -0.3406397827449963));
         // pointsBeforeIssue.add(new Point3D(-0.0470958837642707, -1.2942558333752370, -0.6325393732333888));
         pointsBeforeIssue.add(new Point3D(0.0046484599886485, -0.0503176108042466, 0.0362947448605222));
         troublesomePoint.set(-0.0068611168376205, -0.0525067429256160, 0.0327580791751562);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug5 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug5()
      {
         pointsBeforeIssue.add(new Point3D(0.00987877860785552000, 0.19131147056882847000, 0.03418482283725754500));
         pointsBeforeIssue.add(new Point3D(0.02555035291707655200, 0.11906397969616522000, 0.09162654986018806000));
         pointsBeforeIssue.add(new Point3D(0.03548092653292492000, 0.08553875648964482000, 0.12277926611952411000));
         pointsBeforeIssue.add(new Point3D(0.02835400199038940000, 0.11051604727461928000, 0.09913549801006882000));
         pointsBeforeIssue.add(new Point3D(0.19811056289014100000, 1.53064568455729290000, 0.68554775478046680000));
         troublesomePoint.set(0.03326520528372667000, 0.09790747514458276000, 0.11042444275049243000);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6()
      {
         pointsBeforeIssue.add(new Point3D(-0.16594466242134853000, -0.15167737932767710000, 0.06375940884903786000));
         pointsBeforeIssue.add(new Point3D(-0.15493261840750883000, -0.16646658348557453000, 0.06032647391633983000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000));
         pointsBeforeIssue.add(new Point3D(-0.28134730614168046000, -0.01222890247491981200, 0.10876756841830115000));
         pointsBeforeIssue.add(new Point3D(0.08114060867902650000, -1.60769973787097800000, -0.03136858435585743000));
         pointsBeforeIssue.add(new Point3D(-0.13409442245051250000, -0.19646058810895028000, 0.05184028405466168000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-1.32350866683800360000, 0.45061221257020545000, 0.51166233452409130000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));

         // Could not reproduce the original exception, but manage to highlight a new one by shuffling the pointsBeforeIssue in a given way
         Random random = new Random(478396840718102216L);
         Collections.shuffle(pointsBeforeIssue, random);
         troublesomePoint.set(-0.16885169989877480000, -0.14496535839630130000, 0.07101749538537033000);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6V2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6V2()
      {
         pointsBeforeIssue.add(new Point3D(-0.28134730614168046000, -0.01222890247491981200, 0.10876756841830115000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));
         pointsBeforeIssue.add(new Point3D(-0.16594466242134853000, -0.15167737932767710000, 0.06375940884903786000));
         troublesomePoint.set(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000);
         constructionEpsilon = 0.001;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6V3 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6V3()
      {
         pointsBeforeIssue.add(new Point3D(-0.15493261840750883000, -0.16646658348557453000, 0.06032647391633983000));
         pointsBeforeIssue.add(new Point3D(0.08114060867902650000, -1.60769973787097800000, -0.03136858435585743000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));
         troublesomePoint.set(-1.32350866683800360000, 0.45061221257020545000, 0.51166233452409130000);
         constructionEpsilon = 0.001;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6V4 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6V4()
      {
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));
         pointsBeforeIssue.add(new Point3D(-0.13409442245051250000, -0.19646058810895028000, 0.05184028405466168000));
         pointsBeforeIssue.add(new Point3D(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000));
         troublesomePoint.set(-0.16885169989877480000, -0.14496535839630130000, 0.07101749538537033000);
         constructionEpsilon = 0.001;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug7Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug7Original()
      {
         pointsBeforeIssue.add(new Point3D(0.40484035046382340000, 0.66817675595325240000, 0.37565781894372985000));
         pointsBeforeIssue.add(new Point3D(0.19770492108631288000, -0.11722888133146891000, 0.18345354993551521000));
         pointsBeforeIssue.add(new Point3D(0.13177056177929392000, -0.01608972405889597000, 0.12227200619277923000));
         pointsBeforeIssue.add(new Point3D(0.10804284059622116000, 0.04356860347168567400, 0.10025467521792486000));
         pointsBeforeIssue.add(new Point3D(0.10535595585195023000, 0.05183458577843358400, 0.09786307209807943000));
         pointsBeforeIssue.add(new Point3D(0.10419258778046014000, 0.05592435430920772400, 0.09660853251948309000));
         pointsBeforeIssue.add(new Point3D(0.09875689526332210000, 0.07507951247051448000, 0.09163809842020465000));
         pointsBeforeIssue.add(new Point3D(0.09126555383425328000, 0.10746732352304089000, 0.08468676321119539000));
         pointsBeforeIssue.add(new Point3D(0.09891220007971846000, 0.37479443587413260000, 0.09178220823665961000));

         troublesomePoint.set(0.10219794708629126000, 0.06220152556068970000, 0.09504439824552569000);
         constructionEpsilon = 1.0E-4;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug7Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug7Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D( 0.40484035046382340000,  0.66817675595325240000,  0.37565781894372985000 ));
         pointsBeforeIssue.add(new Point3D(0.19770492108631288000, -0.11722888133146891000, 0.18345354993551521000));
         pointsBeforeIssue.add(new Point3D(0.13177056177929392000, -0.01608972405889597000, 0.12227200619277923000));
         pointsBeforeIssue.add(new Point3D(0.10804284059622116000, 0.04356860347168567400, 0.10025467521792486000));
         pointsBeforeIssue.add(new Point3D(0.10535595585195023000, 0.05183458577843358400, 0.09786307209807943000));
         //      pointsBeforeIssue.add(new Point3D( 0.10419258778046014000,  0.05592435430920772400,  0.09660853251948309000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09875689526332210000,  0.07507951247051448000,  0.09163809842020465000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09126555383425328000,  0.10746732352304089000,  0.08468676321119539000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09891220007971846000,  0.37479443587413260000,  0.09178220823665961000 ));

         troublesomePoint.set(0.10219794708629126000, 0.06220152556068970000, 0.09504439824552569000);
         constructionEpsilon = 1.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8Original()
      {
         pointsBeforeIssue.add(new Point3D(0.25923025651672880000, -1.32459442213312100000, 0.33731064563758340000));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000));
         pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000));
         pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8SimplifiedV2 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8SimplifiedV2()
      {
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug9Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug9Original()
      {
         pointsBeforeIssue.add(new Point3D(0.25923025651672880000, -1.32459442213312100000, 0.33731064563758340000));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000));
         pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000));
         pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug9Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug9Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D( 0.25923025651672880000, -1.32459442213312100000,  0.33731064563758340000 ));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         //      pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600 ));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug10Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug10Original()
      {
         pointsBeforeIssue.add(new Point3D(-0.17317378540777761000, -0.05775495286524568000, 0.14175166115938010000));
         pointsBeforeIssue.add(new Point3D(-0.01897404558870108300, 0.00308339916520594670, 0.01553123340681406000));
         pointsBeforeIssue.add(new Point3D(-0.00274787469423487530, 0.01193080363919030200, 0.00214472895451733870));
         pointsBeforeIssue.add(new Point3D(-0.35181526244719350000, 2.19178801525574850000, 0.28797890948495250000));
         pointsBeforeIssue.add(new Point3D(0.52312335359372390000, 1.17120574040232170000, -0.42820340381521560000));
         pointsBeforeIssue.add(new Point3D(0.35783508614818305000, 0.38370415108791240000, -0.29290644518264797000));
         pointsBeforeIssue.add(new Point3D(0.12404154368939402000, 0.09927405350457708000, -0.10153439118595142000));
         pointsBeforeIssue.add(new Point3D(0.05424289486816125000, 0.04693494504728657000, -0.04440060275503699000));
         pointsBeforeIssue.add(new Point3D(0.01802380347325072000, 0.02392344071554553000, -0.01475341131840446400));
         pointsBeforeIssue.add(new Point3D(-0.00038260858106387020, 0.01322913616138221600, 0.00031318545046343793));
         pointsBeforeIssue.add(new Point3D(-0.00965583175989204200, 0.00808735606657740000, 0.00790374855115949500));
         pointsBeforeIssue.add(new Point3D(-0.00501274681955910000, 0.01064106008930820700, 0.00410473368994557200));
         pointsBeforeIssue.add(new Point3D(-0.00129912505345441960, 0.01133059642016243100, 0.00613679617411944100));
         pointsBeforeIssue.add(new Point3D(-0.00280702931313747700, 0.01130231935193704000, 0.00437573895350351600));
         pointsBeforeIssue.add(new Point3D(-0.00386839007299566440, 0.01128503414358750200, 0.00313791547173392100));

         troublesomePoint.set(-0.00276525936788729600, 0.01161966868636366000, 0.00326143254637401000);
         constructionEpsilon = 1.0E-7;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug10Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug10Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D(-0.17317378540777761000, -0.05775495286524568000, 0.14175166115938010000));
         //      pointsBeforeIssue.add(new Point3D(-0.01897404558870108300, 0.00308339916520594670, 0.01553123340681406000));
         pointsBeforeIssue.add(new Point3D(-0.00274787469423487530, 0.01193080363919030200, 0.00214472895451733870));
         //      pointsBeforeIssue.add(new Point3D(-0.35181526244719350000, 2.19178801525574850000, 0.28797890948495250000));
         //      pointsBeforeIssue.add(new Point3D(0.52312335359372390000, 1.17120574040232170000, -0.42820340381521560000));
         //      pointsBeforeIssue.add(new Point3D(0.35783508614818305000, 0.38370415108791240000, -0.29290644518264797000));
         //      pointsBeforeIssue.add(new Point3D(0.12404154368939402000, 0.09927405350457708000, -0.10153439118595142000));
         //      pointsBeforeIssue.add(new Point3D(0.05424289486816125000, 0.04693494504728657000, -0.04440060275503699000));
         //      pointsBeforeIssue.add(new Point3D(0.01802380347325072000, 0.02392344071554553000, -0.01475341131840446400));
         //      pointsBeforeIssue.add(new Point3D(-0.00038260858106387020, 0.01322913616138221600, 0.00031318545046343793));
         //      pointsBeforeIssue.add(new Point3D(-0.00965583175989204200, 0.00808735606657740000, 0.00790374855115949500));
         pointsBeforeIssue.add(new Point3D(-0.00501274681955910000, 0.01064106008930820700, 0.00410473368994557200));
         //      pointsBeforeIssue.add(new Point3D(-0.00129912505345441960, 0.01133059642016243100, 0.00613679617411944100));
         pointsBeforeIssue.add(new Point3D(-0.00280702931313747700, 0.01130231935193704000, 0.00437573895350351600));
         pointsBeforeIssue.add(new Point3D(-0.00386839007299566440, 0.01128503414358750200, 0.00313791547173392100));

         troublesomePoint.set(-0.00276525936788729600, 0.01161966868636366000, 0.00326143254637401000);
         constructionEpsilon = 1.0E-7;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug11Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug11Original()
      {
         pointsBeforeIssue.add(new Point3D(-0.41935432254164250000, -0.49779783391735880000, 0.51288709338555660000));
         pointsBeforeIssue.add(new Point3D(-0.93298416837861000000, 1.29815954722883200000, 1.14107691890293680000));
         pointsBeforeIssue.add(new Point3D(-0.30925383873489704000, 0.66208031898425670000, 0.37822980220101690000));
         pointsBeforeIssue.add(new Point3D(-0.25512404662567070000, 0.06730540401802787000, 0.31202690348710493000));
         pointsBeforeIssue.add(new Point3D(-0.25507813765207943000, 0.06013191657288902000, 0.31339072703416770000));
         pointsBeforeIssue.add(new Point3D(-0.25531309357128740000, 0.06263846976145082000, 0.31272772115361480000));
         pointsBeforeIssue.add(new Point3D(-0.25641070430508184000, 0.05800263215782730500, 0.31270661888213390000));
         pointsBeforeIssue.add(new Point3D(-0.25641896081503250000, 0.05332514660554210600, 0.31361063528392374000));
         Collections.shuffle(pointsBeforeIssue, new Random(7106315192527041498L));

         troublesomePoint.set(-0.25784327149254240000, 0.05771251668497923000, 0.31159424166064453000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug11Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug11Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-0.41935432254164250000, -0.49779783391735880000, 0.51288709338555660000));
         pointsBeforeIssue.add(new Point3D(-0.93298416837861000000, 1.29815954722883200000, 1.14107691890293680000));
         pointsBeforeIssue.add(new Point3D(-0.30925383873489704000, 0.66208031898425670000, 0.37822980220101690000));
         //      pointsBeforeIssue.add(new Point3D(-0.25512404662567070000,  0.06730540401802787000,  0.31202690348710493000 ));
         pointsBeforeIssue.add(new Point3D(-0.25507813765207943000, 0.06013191657288902000, 0.31339072703416770000));
         pointsBeforeIssue.add(new Point3D(-0.25531309357128740000, 0.06263846976145082000, 0.31272772115361480000));
         pointsBeforeIssue.add(new Point3D(-0.25641070430508184000, 0.05800263215782730500, 0.31270661888213390000));
         pointsBeforeIssue.add(new Point3D(-0.25641896081503250000, 0.05332514660554210600, 0.31361063528392374000));
         Collections.shuffle(pointsBeforeIssue, new Random(7106315192527041498L));

         troublesomePoint.set(-0.25784327149254240000, 0.05771251668497923000, 0.31159424166064453000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug12Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug12Original()
      {
         pointsBeforeIssue.add(new Point3D(-1.06285928606209360000, 0.84863501402184480000, -0.36386991230254034000));
         pointsBeforeIssue.add(new Point3D(-0.24794687685778950000, -0.88150811138554650000, -0.08488462162493815000));
         pointsBeforeIssue.add(new Point3D(-0.14860460603249515000, 0.25024717144708375000, -0.05087479186933352400));
         pointsBeforeIssue.add(new Point3D(-0.04593802791056822400, -0.33056946184021730000, -0.01572688539900857200));
         pointsBeforeIssue.add(new Point3D(-0.05894221041959524000, -0.03259192667445954000, -0.02017886771799948200));
         pointsBeforeIssue.add(new Point3D(-0.09448585378890406000, 0.11211693061937889000, -0.03234723522001137000));
         pointsBeforeIssue.add(new Point3D(-0.07434099339443390000, 0.04041368632546394000, -0.02545064158692189600));
         pointsBeforeIssue.add(new Point3D(-0.06604318267016518000, 0.00405189569758113160, -0.02260988580670941800));
         pointsBeforeIssue.add(new Point3D(-0.06234252506146254000, -0.01423750400223156000, -0.02134296554921921700));
         pointsBeforeIssue.add(new Point3D(-0.06460601902846741000, -0.00286855106762995400, -0.02211787279677335200));
         pointsBeforeIssue.add(new Point3D(-0.06167537739882600000, -0.01770921927976720800, -0.02111456740931594300));
         pointsBeforeIssue.add(new Point3D(-0.06762831802521285000, 0.01144471242318934000, -0.02315255872024968300));
         pointsBeforeIssue.add(new Point3D(-0.06306822611543828000, -0.01052577856589109700, -0.02159139726896253000));
         pointsBeforeIssue.add(new Point3D(-0.06095078718494506600, -0.02154749334084593600, -0.02086667479886617000));
         pointsBeforeIssue.add(new Point3D(-0.06721513866931716000, 0.00953728141767107300, -0.02300866411810875700));
         pointsBeforeIssue.add(new Point3D(-0.06287435077799386000, -0.01148375396021106300, -0.02154246977939222600));
         pointsBeforeIssue.add(new Point3D(-0.06094046529089814000, -0.02202728685290667000, -0.02060449498172178800));
         pointsBeforeIssue.add(new Point3D(-0.06604214004639730000, 0.00928429102395866800, -0.02627429468814690400));
         pointsBeforeIssue.add(new Point3D(-0.06782601695094026000, -0.00492098617148972700, -0.01152854836332983800));
         pointsBeforeIssue.add(new Point3D(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300));

         troublesomePoint.set(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug12Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug12Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-1.06285928606209360000, 0.84863501402184480000, -0.36386991230254034000));
         //      pointsBeforeIssue.add(new Point3D(-0.24794687685778950000, -0.88150811138554650000, -0.08488462162493815000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.14860460603249515000,  0.25024717144708375000, -0.05087479186933352400 ));
         //      pointsBeforeIssue.add(new Point3D(-0.04593802791056822400, -0.33056946184021730000, -0.01572688539900857200 ));
         pointsBeforeIssue.add(new Point3D(-0.05894221041959524000, -0.03259192667445954000, -0.02017886771799948200));
         //      pointsBeforeIssue.add(new Point3D(-0.09448585378890406000,  0.11211693061937889000, -0.03234723522001137000 ));
         pointsBeforeIssue.add(new Point3D(-0.07434099339443390000, 0.04041368632546394000, -0.02545064158692189600));
         //      pointsBeforeIssue.add(new Point3D(-0.06604318267016518000,  0.00405189569758113160, -0.02260988580670941800 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06234252506146254000, -0.01423750400223156000, -0.02134296554921921700 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06460601902846741000, -0.00286855106762995400, -0.02211787279677335200 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06167537739882600000, -0.01770921927976720800, -0.02111456740931594300 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06762831802521285000,  0.01144471242318934000, -0.02315255872024968300 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06306822611543828000, -0.01052577856589109700, -0.02159139726896253000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06095078718494506600, -0.02154749334084593600, -0.02086667479886617000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06721513866931716000,  0.00953728141767107300, -0.02300866411810875700 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06287435077799386000, -0.01148375396021106300, -0.02154246977939222600 ));
         pointsBeforeIssue.add(new Point3D(-0.06094046529089814000, -0.02202728685290667000, -0.02060449498172178800));
         pointsBeforeIssue.add(new Point3D(-0.06604214004639730000, 0.00928429102395866800, -0.02627429468814690400));
         //      pointsBeforeIssue.add(new Point3D(-0.06782601695094026000, -0.00492098617148972700, -0.01152854836332983800 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300 ));

         troublesomePoint.set(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug13Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug13Original()
      {
         pointsBeforeIssue.add(new Point3D(-1.16485579659865150000, 1.50478742011243050000, 0.92998233563902600000));
         pointsBeforeIssue.add(new Point3D(-0.49671459562836695000, -0.57321041192247600000, 0.39656050228474937000));
         pointsBeforeIssue.add(new Point3D(-0.28684916277252004000, 0.75215556219304490000, 0.22901088284939053000));
         pointsBeforeIssue.add(new Point3D(-0.23434852727844335000, 0.04865437022396490500, 0.18709611214396826000));
         pointsBeforeIssue.add(new Point3D(-0.22769504113460493000, 0.09534067425821080000, 0.18178418889796938000));
         pointsBeforeIssue.add(new Point3D(-0.23084361780005236000, 0.07195594672506622000, 0.18429790835562376000));
         pointsBeforeIssue.add(new Point3D(-0.23255161132903990000, 0.06029420820407039000, 0.18566151389030200000));
         pointsBeforeIssue.add(new Point3D(-0.23343896451436674000, 0.05447148220883074600, 0.18636994732063494000));
         pointsBeforeIssue.add(new Point3D(-0.23084251150272717000, 0.07196370955534642000, 0.18429702512443302000));
         pointsBeforeIssue.add(new Point3D(-0.23255102857838360000, 0.06029808252032909000, 0.18566104864145722000));
         pointsBeforeIssue.add(new Point3D(-0.23343866574888150000, 0.05447341752291190400, 0.18636970879614645000));
         pointsBeforeIssue.add(new Point3D(-0.23084361798631325000, 0.07195594541800332000, 0.18429790850435868000));
         pointsBeforeIssue.add(new Point3D(-0.23255161142718340000, 0.06029420755173304000, 0.18566151396861086000));
         pointsBeforeIssue.add(new Point3D(-0.23343896456459690000, 0.05447148188295081000, 0.18636994736089696000));
         pointsBeforeIssue.add(new Point3D(-0.23084251131672584000, 0.07196371086271980000, 0.18429702497529600000));
         pointsBeforeIssue.add(new Point3D(-0.23255102847977416000, 0.06029808317281127000, 0.18566104856369010000));
         pointsBeforeIssue.add(new Point3D(-0.23343866570020700000, 0.05447341784881787000, 0.18636970875392822000));
         pointsBeforeIssue.add(new Point3D(-0.23084361816609666000, 0.07195594411052708000, 0.18429790866132790000));
         pointsBeforeIssue.add(new Point3D(-0.23255161153516546000, 0.06029420689919168000, 0.18566151403466002000));
         pointsBeforeIssue.add(new Point3D(-0.23343896458049573000, 0.05447148155701498000, 0.18636994744417840000));
         pointsBeforeIssue.add(new Point3D(-0.23084251126809020000, 0.07196371217031738000, 0.18429702465403558000));
         pointsBeforeIssue.add(new Point3D(-0.23255102817507890000, 0.06029808382542023400, 0.18566104874401880000));
         pointsBeforeIssue.add(new Point3D(-0.23343866637276056000, 0.05447341817483153400, 0.18636970780829720000));
         pointsBeforeIssue.add(new Point3D(-0.23084361546016796000, 0.07195594280261897000, 0.18429791243294170000));
         pointsBeforeIssue.add(new Point3D(-0.23255161597315000000, 0.06029420624642717000, 0.18566150867720305000));
         pointsBeforeIssue.add(new Point3D(-0.23343894943555432000, 0.05447148123095147000, 0.18636996651731620000));
         pointsBeforeIssue.add(new Point3D(-0.23084257187954450000, 0.07196371347842823000, 0.18429694835241683000));
         pointsBeforeIssue.add(new Point3D(-0.23255093689286900000, 0.06029808447907076000, 0.18566116287855428000));
         pointsBeforeIssue.add(new Point3D(-0.23343898546925080000, 0.05447341850844744600, 0.18636930801633494000));
         pointsBeforeIssue.add(new Point3D(-0.23084233870854210000, 0.07195594146460049000, 0.18429951203091433000));
         pointsBeforeIssue.add(new Point3D(-0.23255353201902773000, 0.06029420592756607000, 0.18565910882543690000));
         pointsBeforeIssue.add(new Point3D(-0.23343224039050580000, 0.05447148421488279000, 0.18637836911135863000));
         pointsBeforeIssue.add(new Point3D(-0.23086941605680810000, 0.07196370167909760000, 0.18426332918815225000));
         pointsBeforeIssue.add(new Point3D(-0.23251065898995815000, 0.06029823265925538000, 0.18571157034205898000));
         pointsBeforeIssue.add(new Point3D(-0.23357967126484824000, 0.05447487912998916000, 0.18619266304844595000));
         pointsBeforeIssue.add(new Point3D(-0.23027969794173497000, 0.07195015744675376000, 0.18500649211601283000));
         pointsBeforeIssue.add(new Point3D(-0.23190458293936178000, 0.06320641868777910000, 0.18557954893731465000));
         pointsBeforeIssue.add(new Point3D(-0.23378371205299409000, 0.05595052063483186000, 0.18547193991612554000));
         pointsBeforeIssue.add(new Point3D(-0.23118909226302153000, 0.05911705963689206600, 0.18773635485865492000));
         pointsBeforeIssue.add(new Point3D(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000));

         troublesomePoint.set(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug13Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug13Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-1.16485579659865150000, 1.50478742011243050000, 0.92998233563902600000));
         pointsBeforeIssue.add(new Point3D(-0.49671459562836695000, -0.57321041192247600000, 0.39656050228474937000));
         pointsBeforeIssue.add(new Point3D(-0.28684916277252004000, 0.75215556219304490000, 0.22901088284939053000));
         pointsBeforeIssue.add(new Point3D(-0.23434852727844335000, 0.04865437022396490500, 0.18709611214396826000));
         pointsBeforeIssue.add(new Point3D(-0.22769504113460493000, 0.09534067425821080000, 0.18178418889796938000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361780005236000, 0.07195594672506622000, 0.18429790835562376000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161132903990000, 0.06029420820407039000, 0.18566151389030200000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896451436674000, 0.05447148220883074600, 0.18636994732063494000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251150272717000, 0.07196370955534642000, 0.18429702512443302000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102857838360000, 0.06029808252032909000, 0.18566104864145722000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866574888150000, 0.05447341752291190400, 0.18636970879614645000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361798631325000, 0.07195594541800332000, 0.18429790850435868000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161142718340000, 0.06029420755173304000, 0.18566151396861086000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896456459690000, 0.05447148188295081000, 0.18636994736089696000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251131672584000, 0.07196371086271980000, 0.18429702497529600000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102847977416000, 0.06029808317281127000, 0.18566104856369010000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866570020700000, 0.05447341784881787000, 0.18636970875392822000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361816609666000, 0.07195594411052708000, 0.18429790866132790000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161153516546000, 0.06029420689919168000, 0.18566151403466002000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896458049573000, 0.05447148155701498000, 0.18636994744417840000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251126809020000, 0.07196371217031738000, 0.18429702465403558000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102817507890000, 0.06029808382542023400, 0.18566104874401880000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866637276056000, 0.05447341817483153400, 0.18636970780829720000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361546016796000, 0.07195594280261897000, 0.18429791243294170000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161597315000000, 0.06029420624642717000, 0.18566150867720305000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343894943555432000, 0.05447148123095147000, 0.18636996651731620000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084257187954450000, 0.07196371347842823000, 0.18429694835241683000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255093689286900000, 0.06029808447907076000, 0.18566116287855428000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343898546925080000, 0.05447341850844744600, 0.18636930801633494000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084233870854210000, 0.07195594146460049000, 0.18429951203091433000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255353201902773000, 0.06029420592756607000, 0.18565910882543690000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343224039050580000, 0.05447148421488279000, 0.18637836911135863000));
         //      pointsBeforeIssue.add(new Point3D(-0.23086941605680810000, 0.07196370167909760000, 0.18426332918815225000));
         //      pointsBeforeIssue.add(new Point3D(-0.23251065898995815000, 0.06029823265925538000, 0.18571157034205898000));
         //      pointsBeforeIssue.add(new Point3D(-0.23357967126484824000, 0.05447487912998916000, 0.18619266304844595000));
         //      pointsBeforeIssue.add(new Point3D(-0.23027969794173497000, 0.07195015744675376000, 0.18500649211601283000));
         pointsBeforeIssue.add(new Point3D(-0.23190458293936178000, 0.06320641868777910000, 0.18557954893731465000));
         pointsBeforeIssue.add(new Point3D(-0.23378371205299409000, 0.05595052063483186000, 0.18547193991612554000));
         pointsBeforeIssue.add(new Point3D(-0.23118909226302153000, 0.05911705963689206600, 0.18773635485865492000));
         //      pointsBeforeIssue.add(new Point3D(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000));

         troublesomePoint.set(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrity_20190228_220911 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * The main thing this dataset reveals is that new faces are created to only have 3 vertices which
       * is a limitation and would definitely prevent this bug.
       */
      public DatasetGJKFaceNormalIntegrity_20190228_220911()
      {
         pointsBeforeIssue.add(new Point3D(0.69290152806624020000, 0.50768999063042180000, 0.06363179078759229000));
         pointsBeforeIssue.add(new Point3D(0.18276181406493613000, -0.50024222323255470000, 0.01678371463402106300));
         pointsBeforeIssue.add(new Point3D(0.12994643325986270000, 0.16086647763109743000, 0.01193347672051159300));
         pointsBeforeIssue.add(new Point3D(0.06342912096006117000, -0.17717417522993928000, 0.00582493816406426400));
         pointsBeforeIssue.add(new Point3D(0.07338575205979947000, -0.00352996758700958870, 0.00673929358316116560));
         pointsBeforeIssue.add(new Point3D(0.06245096874381456000, -0.09000765157352504000, 0.00573511071432963900));
         pointsBeforeIssue.add(new Point3D(0.06643690653972065000, -0.04657990542776806000, 0.00610115458874538300));
         pointsBeforeIssue.add(new Point3D(0.08928878518501723000, 0.05989188268082375000, 0.00819972978617866700));
         pointsBeforeIssue.add(new Point3D(0.07558237236266321000, 0.00714957099078997200, 0.00694101760581999900));
         pointsBeforeIssue.add(new Point3D(0.06278639727126109000, -0.08456553041509965000, 0.00576591439572837800));
         pointsBeforeIssue.add(new Point3D(0.06751492505399428000, -0.03847309446811371000, 0.00620015314159383400));
         pointsBeforeIssue.add(new Point3D(0.09049802033207752000, 0.06379683716838570000, 0.00831077846327110700));
         pointsBeforeIssue.add(new Point3D(0.07689823545984276000, 0.01313965058830352000, 0.00706185833392369840));
         pointsBeforeIssue.add(new Point3D(0.06299554871702032000, -0.08150567262327739000, 0.00578512157253641100));
         pointsBeforeIssue.add(new Point3D(0.06816703680253616000, -0.03391935874795497000, 0.00626003904916216900));
         pointsBeforeIssue.add(new Point3D(0.09118922701245419000, 0.06598990756695286000, 0.00837425462676999500));
         pointsBeforeIssue.add(new Point3D(0.07766374510111382000, 0.01650336615144861000, 0.00713215799506905500));
         pointsBeforeIssue.add(new Point3D(0.06311967180957340000, -0.07978496015632038000, 0.00579652034065247800));
         pointsBeforeIssue.add(new Point3D(0.06854830123360611000, -0.03136000744856276600, 0.00629505190098900200));
         pointsBeforeIssue.add(new Point3D(0.09158151048933016000, 0.06722239871967348000, 0.00841028044705882700));
         pointsBeforeIssue.add(new Point3D(0.07810234996807222000, 0.01839360256423040700, 0.00717243643491638500));
         pointsBeforeIssue.add(new Point3D(0.06319153925915186000, -0.07881721039423031000, 0.00580312250506209700));
         pointsBeforeIssue.add(new Point3D(0.06876733232926746000, -0.02992106557740965000, 0.00631516224016659400));
         pointsBeforeIssue.add(new Point3D(0.09180326716383555000, 0.06791532097154179000, 0.00843067884245729500));
         pointsBeforeIssue.add(new Point3D(0.07835159687695523000, 0.01945626539304043400, 0.00719531862188327300));
         pointsBeforeIssue.add(new Point3D(0.06323260822286730000, -0.07827290003949239000, 0.00580694502532530500));
         pointsBeforeIssue.add(new Point3D(0.06889199109869065000, -0.02911188382893079000, 0.00632651417617557900));
         pointsBeforeIssue.add(new Point3D(0.09192828327024699000, 0.06830497727632491000, 0.00844294715850857400));
         pointsBeforeIssue.add(new Point3D(0.07849260822028062000, 0.02005382338668646000, 0.00720816013487235450));
         pointsBeforeIssue.add(new Point3D(0.06325584682864169000, -0.07796674112778025000, 0.00580985143225110100));
         pointsBeforeIssue.add(new Point3D(0.06896269980927255000, -0.02865679034622081700, 0.00633151548305721500));
         pointsBeforeIssue.add(new Point3D(0.09199763627676469000, 0.06852412313249402000, 0.00846160465054612800));
         pointsBeforeIssue.add(new Point3D(0.07857228186350942000, 0.02038989034088247000, 0.00721430334882267040));
         pointsBeforeIssue.add(new Point3D(0.06326826137164637000, -0.07779453119896886000, 0.00581935590873783300));
         pointsBeforeIssue.add(new Point3D(0.06900403467768462000, -0.02840082445718784800, 0.00631891665373540700));
         pointsBeforeIssue.add(new Point3D(0.09202500485407222000, 0.06864736217071214000, 0.00859938720213293500));
         pointsBeforeIssue.add(new Point3D(0.07861803431238706000, 0.02057902172424973300, 0.00720874120642356300));
         pointsBeforeIssue.add(new Point3D(0.06326937390894047000, -0.07769760267388713000, 0.00588895865962191200));
         pointsBeforeIssue.add(new Point3D(0.06903900103459407000, -0.02825692897117966500, 0.00618482993385105500));
         pointsBeforeIssue.add(new Point3D(0.09194493958676875000, 0.06871527603606316000, 0.00972599825232489700));
         pointsBeforeIssue.add(new Point3D(0.07865135114561148000, 0.02069355532833770700, 0.00714465985551715500));
         pointsBeforeIssue.add(new Point3D(0.06323064952435398000, -0.07763869526608258000, 0.00636294866345932200));
         pointsBeforeIssue.add(new Point3D(0.06913858511331608000, -0.02817792609644004200, 0.00524622987493783550));
         pointsBeforeIssue.add(new Point3D(0.09128845469645783000, 0.06868421363121194000, 0.01749622914526949300));
         pointsBeforeIssue.add(new Point3D(0.07879569557827140000, 0.02115722735664338200, 0.00678618050052637300));
         pointsBeforeIssue.add(new Point3D(0.06301737621497602000, -0.07739422362078208000, 0.00897688092849968600));
         pointsBeforeIssue.add(new Point3D(0.06966918356725210000, -0.02809655780659242600, -0.00004337270235066359));
         pointsBeforeIssue.add(new Point3D(0.07492645441585288000, 0.01477008624690665000, 0.04604939960311582600));
         troublesomePoint.set(0.08045526764839975000, 0.02285563605061801000, -0.00538019769538478100);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190302_160115 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlighted a bug in Face3D which was caused by poor management of the
       * constructionEpsilon.
       */
      public ConvexPolytope3DTroublesomeDataset_20190302_160115()
      {
         pointsBeforeIssue.add(new Point3D(-0.97083141752365850000, 0.19270232253646270000, -1.10314697851813780000));
         pointsBeforeIssue.add(new Point3D(0.08735078445224298000, -1.63277627043767500000, 0.09925590808079998000));
         pointsBeforeIssue.add(new Point3D(-0.16861590228562445000, -0.35729391130264250000, -0.19159672810234674000));
         pointsBeforeIssue.add(new Point3D(-0.35048289420867120000, -0.10822871051009664000, -0.39825054977597630000));
         pointsBeforeIssue.add(new Point3D(-0.25480704371353974000, -0.22482735589148084000, -0.28953494427973680000));
         pointsBeforeIssue.add(new Point3D(-0.21045139648732920000, -0.28918214661603340000, -0.23913402262165895000));
         pointsBeforeIssue.add(new Point3D(-0.18920976292038977000, -0.32278223301503760000, -0.21499734609348364000));
         pointsBeforeIssue.add(new Point3D(-0.26251174735750926000, -0.21437817652630936000, -0.29828972949971044000));
         pointsBeforeIssue.add(new Point3D(-0.21408984752431032000, -0.28361727533457450000, -0.24326836169998278000));
         pointsBeforeIssue.add(new Point3D(-0.19097303137237798000, -0.31991844716892740000, -0.21700093212295768000));
         pointsBeforeIssue.add(new Point3D(-0.25532530152410793000, -0.22411807334799222000, -0.29012383595290490000));
         pointsBeforeIssue.add(new Point3D(-0.21069657245417367000, -0.28880546074366414000, -0.23941261386019497000));
         pointsBeforeIssue.add(new Point3D(-0.18932870793247086000, -0.32258861213242257000, -0.21513250223734293000));
         pointsBeforeIssue.add(new Point3D(-0.26202498230546567000, -0.21503232293794655000, -0.29773662276369195000));
         pointsBeforeIssue.add(new Point3D(-0.21386038394086704000, -0.28396664073018063000, -0.24300762430112366000));
         pointsBeforeIssue.add(new Point3D(-0.19086194785992644000, -0.32009845157381284000, -0.21687470892949090000));
         pointsBeforeIssue.add(new Point3D(-0.25577615488337163000, -0.22350179846019480000, -0.29063613655908904000));
         pointsBeforeIssue.add(new Point3D(-0.21090980920004387000, -0.28847804640993530000, -0.23965491284606855000));
         pointsBeforeIssue.add(new Point3D(-0.18943214279826670000, -0.32242029058484610000, -0.21525003435633305000));
         pointsBeforeIssue.add(new Point3D(-0.26160192186689590000, -0.21560151221163665000, -0.29725590295121050000));
         pointsBeforeIssue.add(new Point3D(-0.21366090718484270000, -0.28427052340751540000, -0.24278096073773303000));
         pointsBeforeIssue.add(new Point3D(-0.19076536817251855000, -0.32025499828839680000, -0.21676496605945483000));
         pointsBeforeIssue.add(new Point3D(-0.25616834945327020000, -0.22296627726528990000, -0.29108178461917344000));
         pointsBeforeIssue.add(new Point3D(-0.21109526438785820000, -0.28819344177455190000, -0.23986564255056098000));
         pointsBeforeIssue.add(new Point3D(-0.18952208758489730000, -0.32227395694270733000, -0.21535224252176488000));
         pointsBeforeIssue.add(new Point3D(-0.26123421207556220000, -0.21609673816089336000, -0.29683805997600940000));
         pointsBeforeIssue.add(new Point3D(-0.21348747937756680000, -0.28453483575594035000, -0.24258392070106327000));
         pointsBeforeIssue.add(new Point3D(-0.19068143614879580000, -0.32039114237603810000, -0.21666951473614948000));
         pointsBeforeIssue.add(new Point3D(-0.25650935309177320000, -0.22250088499806486000, -0.29146956412627790000));
         pointsBeforeIssue.add(new Point3D(-0.21125677344005106000, -0.28794603693798404000, -0.24004871998837785000));
         pointsBeforeIssue.add(new Point3D(-0.18959954084944286000, -0.32214673477981360000, -0.21544179390074947000));
         pointsBeforeIssue.add(new Point3D(-0.26091744042974535000, -0.21652758125062954000, -0.29647234178060020000));
         pointsBeforeIssue.add(new Point3D(-0.21333273792329100000, -0.28476472339561754000, -0.24241610814400594000));
         pointsBeforeIssue.add(new Point3D(-0.19062152548312090000, -0.32050954055667260000, -0.21657502502991666000));
         pointsBeforeIssue.add(new Point3D(-0.25675701987444710000, -0.22209640957841460000, -0.29184993243613800000));
         pointsBeforeIssue.add(new Point3D(-0.21146931819101095000, -0.28773093222510954000, -0.24014454369490168000));
         pointsBeforeIssue.add(new Point3D(-0.18941766249809594000, -0.32203584875844776000, -0.21573941078587810000));
         pointsBeforeIssue.add(new Point3D(-0.26157550750578480000, -0.21690377434253066000, -0.29533185660364370000));
         pointsBeforeIssue.add(new Point3D(-0.21190304601336580000, -0.28495523356869720000, -0.24342425492789554000));
         pointsBeforeIssue.add(new Point3D(-0.19493169094055962000, -0.32052879613810250000, -0.21277662508582130000));
         troublesomePoint.set(-0.24069598189087021000, -0.22216401323722800000, -0.30617162217711513000);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_111711 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This dataset highlighted a bug when destroying faces, newly faces were also destroyed.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_111711()
      {
         pointsBeforeIssue.add(new Point3D(0.42651861137013536000, 0.29801909203948130000, -0.50881487394525300000));
         pointsBeforeIssue.add(new Point3D(-0.20516265604483996000, -1.22116838270320940000, 0.24474854834209914000));
         pointsBeforeIssue.add(new Point3D(-0.15474743628494347000, -0.19414762426986742000, 0.18460577144275080000));
         pointsBeforeIssue.add(new Point3D(0.09567299040224675000, 0.16701603647134378000, -0.11413298096209190000));
         pointsBeforeIssue.add(new Point3D(1.25251552911340270000, -0.77979707379684860000, 0.18358441780536028000));
         pointsBeforeIssue.add(new Point3D(-0.42349443545899990000, -0.75484408613651910000, -1.17104055750426640000));
         pointsBeforeIssue.add(new Point3D(0.66343852027333500000, -0.21204323532349700000, 0.37617790738248347000));
         pointsBeforeIssue.add(new Point3D(-0.48038709090787757000, -0.20046079757003776000, -0.56864140285513080000));
         pointsBeforeIssue.add(new Point3D(0.26413164476960427000, -0.03607115324095866400, 0.25419883203236570000));
         pointsBeforeIssue.add(new Point3D(-0.28904627974853190000, -0.03294277079647378600, -0.20572540856203970000));
         pointsBeforeIssue.add(new Point3D(0.02663658174361427400, 0.00806765192311997800, 0.10847859247578451000));
         pointsBeforeIssue.add(new Point3D(-0.10798349555768538000, 0.05234735820899627500, -0.08040989111998509000));
         pointsBeforeIssue.add(new Point3D(0.01968789268608772700, 0.08783884515805285000, -0.01547486359495420200));
         troublesomePoint.set(-0.12185888684659330000, -0.04755688773206634400, 0.06109223839806776600);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_120656 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_120656()
      {
         pointsBeforeIssue.add(new Point3D(-0.90845250809078680000, 2.27040165191556880000, 0.85520029030598930000));
         pointsBeforeIssue.add(new Point3D(-0.35028001093434450000, -0.71632092412883900000, 0.32974708569960800000));
         pointsBeforeIssue.add(new Point3D(0.21414664741334005000, 1.07437997978164650000, -0.20159366990006766000));
         pointsBeforeIssue.add(new Point3D(0.15515353708065260000, 0.04631896555688220000, -0.14605865333811735000));
         pointsBeforeIssue.add(new Point3D(0.18259557865711085000, 0.67866202579018810000, 2.01418653258046800000));
         pointsBeforeIssue.add(new Point3D(-1.99950059483868440000, 0.67866202579018850000, -0.30378595196848980000));
         pointsBeforeIssue.add(new Point3D(0.46399006455839110000, -0.09965128578182691000, 1.06559233195128210000));
         pointsBeforeIssue.add(new Point3D(-1.03565694775630700000, -0.09965128578182703000, -0.52743576337442480000));
         pointsBeforeIssue.add(new Point3D(0.24013142981869978000, -0.34191292573890190000, 0.43948795795885110000));
         pointsBeforeIssue.add(new Point3D(-0.42419935030018024000, -0.34191292573890190000, -0.26620984200911800000));
         pointsBeforeIssue.add(new Point3D(-0.03598353834151968000, -0.36912442321079680000, 0.03387423355853442000));
         pointsBeforeIssue.add(new Point3D(0.23524026643487567000, -0.16034770641498058000, 0.13293680366100574000));
         pointsBeforeIssue.add(new Point3D(-0.11850174993658597000, -0.16034770641498020000, -0.24283227129424856000));
         pointsBeforeIssue.add(new Point3D(0.07132931048778474000, -0.17159446392155820000, -0.06714808588583293000));
         troublesomePoint.set(0.20140763882017088000, -0.06242105790225444000, -0.01234876003148449600);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_122006 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlights an edge-case triggered during the expansion of a new face which would result
       * in rejecting the vertex being added.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_122006()
      {
         pointsBeforeIssue.add(new Point3D(-0.72399987695997490000, 0.78925182235832480000, 0.68035162188417670000));
         pointsBeforeIssue.add(new Point3D(-0.10066480308695103000, -0.98268929676327770000, 0.09459595812976185000));
         pointsBeforeIssue.add(new Point3D(-0.00707474541866726100, 0.17173375187195128000, 0.00664822560498046200));
         pointsBeforeIssue.add(new Point3D(0.06720565503243336000, -0.42396129483605040000, -0.06315398366239744000));
         pointsBeforeIssue.add(new Point3D(-1.47209228609775970000, -0.30317489537983360000, -0.11573498208463157000));
         pointsBeforeIssue.add(new Point3D(0.02409253217780982200, -0.30317489537983240000, 1.47643822585298510000));
         pointsBeforeIssue.add(new Point3D(0.36013545166180660000, -0.17032436693446074000, 0.70028801960523150000));
         pointsBeforeIssue.add(new Point3D(-0.67657142012428340000, -0.17032436693445230000, -0.40292923375859380000));
         pointsBeforeIssue.add(new Point3D(0.26227976702423017000, -0.12906871934200104000, 0.24407689541467560000));
         pointsBeforeIssue.add(new Point3D(-0.22731783509811143000, -0.12906871934199704000, -0.27693105443749210000));
         pointsBeforeIssue.add(new Point3D(0.09636164594442587000, -0.11914561874941587000, -0.01716994101479385500));
         pointsBeforeIssue.add(new Point3D(-0.07201500779408909000, -0.00387022171032114230, -0.14349822871148532000));
         pointsBeforeIssue.add(new Point3D(0.03578107824730514000, 0.02537458904681466600, -0.03251882217663682000));
         pointsBeforeIssue.add(new Point3D(0.15793238320759340000, 0.05596883793175411000, 0.14499435712828979000));
         troublesomePoint.set(0.01219650720536491500, -0.07685272305708180000, -0.09436722510817275000);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_142536 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlights the following issue: the vertex is on the support line of an edge. The
       * edge's face is an in-plane face, but the twin's face is not. This would create a broken polytope
       * when poorly handled.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_142536()
      {
         pointsBeforeIssue.add(new Point3D(-0.77952021930564740000, 0.68175091439513100000, 0.88022153629260390000));
         pointsBeforeIssue.add(new Point3D(0.00755257946139870100, -1.37890050133029280000, -0.00852824972571714000));
         pointsBeforeIssue.add(new Point3D(0.00882239922256677900, -0.00550188742598167750, -0.00996210951961151700));
         pointsBeforeIssue.add(new Point3D(0.24782431072536026000, -0.69054427019503040000, 1.79003321475398370000));
         pointsBeforeIssue.add(new Point3D(-1.80686474933665500000, -0.69054427019503040000, -0.02959014216877598000));
         pointsBeforeIssue.add(new Point3D(0.23712437564132222000, 0.20769595225738152000, 1.08708464074358800000));
         pointsBeforeIssue.add(new Point3D(-1.10777895062942820000, 0.20769595225738152000, -0.10395567423794305000));
         pointsBeforeIssue.add(new Point3D(-0.50044462323115240000, 0.48121404765904590000, 0.22273136156388870000));
         pointsBeforeIssue.add(new Point3D(0.04037873351940657000, 0.34514418372584005000, 0.50832207780856380000));
         pointsBeforeIssue.add(new Point3D(-0.15433294366209777000, 0.30325456009572970000, 0.16979995093961175000));
         pointsBeforeIssue.add(new Point3D(0.11718757317754325000, 0.14369068070229263000, 0.26122133011282433000));
         pointsBeforeIssue.add(new Point3D(-0.02241124340936162500, 0.15722149478681585000, 0.11127857352399390000));
         troublesomePoint.set(-0.07807324306319463000, 0.15742988410454184000, 0.06040079563670270000);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_154201 extends ConvexPolytope3DTroublesomeDataset
   {
      /**
       * This data highlights an edge-case triggered going from single face to tetrahedron with the
       * original face big enough to have made it through the epsilon tests, but small enough to trigger
       * new face expansion resulting a polytope with 3 faces instead of 4.
       */
      public ConvexPolytope3DTroublesomeDataset_20190303_154201()
      {
         pointsBeforeIssue.add(new Point3D(0.00513254314630662300, 0.01135304298811235000, 0.01418412475941843700));
         pointsBeforeIssue.add(new Point3D(0.00021120765251825270, -0.01763511715856947000, 0.00058368641199196820));
         pointsBeforeIssue.add(new Point3D(-0.00037900022155279434, 0.00133264215432005170, -0.00104739234977868630));
         troublesomePoint.set(0.02215470614908077200, -0.00674925963584982500, 0.00802463395282543300);
         constructionEpsilon = 0.01;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_165341 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_165341()
      {
         pointsBeforeIssue.add(new Point3D(0.87891217943452820000, 0.79872975655805370000, -0.77025592772239560000));
         pointsBeforeIssue.add(new Point3D(0.10414943353234363000, -1.15315213972416180000, -0.09127387289004246000));
         pointsBeforeIssue.add(new Point3D(0.04898639323837373000, 0.13336066496869303000, -0.04293040949082477000));
         pointsBeforeIssue.add(new Point3D(-0.05799225023803511500, -0.52009668762978620000, 0.05082290990272153600));
         pointsBeforeIssue.add(new Point3D(-0.03803986464642317000, -0.18366082728861400000, 0.03333715463190490000));
         pointsBeforeIssue.add(new Point3D(-0.00262433603527673100, -0.02121994150400774400, 0.00229990030233562240));
         pointsBeforeIssue.add(new Point3D(0.02120909388714043000, 0.05723440083901532000, -0.01858710188571258000));
         pointsBeforeIssue.add(new Point3D(0.00879211227490628300, 0.01827592444549197600, -0.00770518450552462600));
         pointsBeforeIssue.add(new Point3D(0.00295828990973345800, -0.00140765941618214980, -0.00259193454719702800));
         pointsBeforeIssue.add(new Point3D(0.00004159554989802139, -0.01129806815557932200, -0.00022538704045779734));
         pointsBeforeIssue.add(new Point3D(0.00165459812629076270, -0.00635201355375258900, -0.00121449960979916340));
         pointsBeforeIssue.add(new Point3D(0.00156308424995355020, -0.00391360267100382400, -0.00272798489336986700));
         troublesomePoint.set(0.00014363976288112035, -0.01196889612053064800, 0.00027199536220018360);
         constructionEpsilon = 1.0E-4;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_172836 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_172836()
      {
         pointsBeforeIssue.add(new Point3D(0.06747410667733611000, 0.01097006791540566800, -0.01855584631406093000));
         pointsBeforeIssue.add(new Point3D(-0.23450880249566974000, -0.35514590183382583000, 0.06449154368524224000));
         pointsBeforeIssue.add(new Point3D(-0.16480266637687052000, -0.09997082136786029000, 0.04532187382725045000));
         pointsBeforeIssue.add(new Point3D(-0.06038812163569981500, -0.01809778776209314300, 0.01660715138661117400));
         pointsBeforeIssue.add(new Point3D(0.00202511726880032200, 0.00361784865624836800, -0.00055692126444689680));
         pointsBeforeIssue.add(new Point3D(0.15153572105567930000, -0.30604894915001346000, 0.28711505869528310000));
         pointsBeforeIssue.add(new Point3D(-0.07369353077212262000, -0.25285670143166930000, -0.29738077364433600000));
         pointsBeforeIssue.add(new Point3D(0.05779532695296065000, -0.09290449420847424000, 0.21591551597107142000));
         pointsBeforeIssue.add(new Point3D(-0.06095504775507306000, -0.07133398996452434000, -0.18860259696650689000));
         pointsBeforeIssue.add(new Point3D(0.00884534586488244800, -0.02740019291422951500, 0.12079477706083219000));
         pointsBeforeIssue.add(new Point3D(-0.05169272294775219000, -0.02161279109018687800, -0.09201601161740780000));
         pointsBeforeIssue.add(new Point3D(-0.01341711532192288400, -0.01019306955203092000, 0.06161251335417228000));
         pointsBeforeIssue.add(new Point3D(-0.04100666320760343000, -0.00873109999379462200, -0.03686093630059778000));
         pointsBeforeIssue.add(new Point3D(-0.02491492804702956400, -0.00595864339012780500, 0.02516064751564628400));
         pointsBeforeIssue.add(new Point3D(-0.02883833506108163200, -0.00423312880939052950, -0.00705883193530876250));
         pointsBeforeIssue.add(new Point3D(-0.01562781722835876500, -0.00126061520025416700, 0.00788652333063244400));
         pointsBeforeIssue.add(new Point3D(-0.01257580112268012800, 0.00054105599367043620, -0.00877636839705048200));
         pointsBeforeIssue.add(new Point3D(-0.00817348912662128900, 0.00120252498154777000, 0.00080984691360735940));
         pointsBeforeIssue.add(new Point3D(-0.00064066905671167350, 0.00154563211405506220, 0.01675715532566424000));
         pointsBeforeIssue.add(new Point3D(-0.00200694895364611500, 0.00215160495631450100, 0.00763305473354353200));
         pointsBeforeIssue.add(new Point3D(-0.00281060462236036600, 0.00239234492507844900, 0.00206173472556359100));
         pointsBeforeIssue.add(new Point3D(-0.00408405263766370400, 0.00256756996996077000, -0.00687975352391062560));
         pointsBeforeIssue.add(new Point3D(-0.00215324445034560660, 0.00281684027960105170, -0.00258671120635789450));
         pointsBeforeIssue.add(new Point3D(-0.00078689974031906360, 0.00297654768486216300, 0.00001118064050609213));
         pointsBeforeIssue.add(new Point3D(0.00151196108896307910, 0.00318524073757953100, 0.00434106035620457350));
         pointsBeforeIssue.add(new Point3D(0.00104652179994238480, 0.00326303258118582070, 0.00180419067017462130));
         troublesomePoint.set(0.00244062696426103340, 0.00376191324193225760, -0.00155780991298043460);
         constructionEpsilon = 1.0E-5;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190303_180109 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190303_180109()
      {
         pointsBeforeIssue.add(new Point3D(-0.00018629439324591157, 0.00049034938467718000, -0.00087149249641926830));
         pointsBeforeIssue.add(new Point3D(0.00004798972671832269, -0.00140551706489366880, 0.00022449782847200340));
         pointsBeforeIssue.add(new Point3D(0.00004389856088266031, -0.00013788428156769061, 0.00020535919383435353));
         pointsBeforeIssue.add(new Point3D(-0.00005378436125835773, 0.00032224543715053680, -0.00025160535668600215));
         pointsBeforeIssue.add(new Point3D(0.00106464605894840410, -0.00078885291272845490, -0.00113889949474199610));
         pointsBeforeIssue.add(new Point3D(-0.00143723484544022730, -0.00078885291272867700, -0.00060408549809654040));
         pointsBeforeIssue.add(new Point3D(0.00081800251973424000, -0.00013347834258559121, -0.00042626176850790730));
         pointsBeforeIssue.add(new Point3D(-0.00092078735873357690, -0.00013347834258581326, -0.00005456974967532613));
         pointsBeforeIssue.add(new Point3D(0.00041022156481829164, 0.00006978320198436361, -0.00013442312928102496));
         pointsBeforeIssue.add(new Point3D(-0.00042932789446292750, 0.00006978320198414156, 0.00004504296797114282));
         pointsBeforeIssue.add(new Point3D(0.00009218181317455221, 0.00012020073833973743, -0.00001567068107144465));
         pointsBeforeIssue.add(new Point3D(-0.00015293051115145007, 0.00003934793020243799, 0.00010284252441705721));
         troublesomePoint.set(-0.00034597504967026627, 0.00038883490048968740, -0.00039829906847554940);
         constructionEpsilon = 1.0E-5;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190317_143836 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190317_143836()
      {
         pointsBeforeIssue.add(new Point3D(0.70748115689297640000, 0.96081283193249120000, 1.33209407268086260000));
         pointsBeforeIssue.add(new Point3D(0.02109978554369074000, -1.70635128569457660000, 0.03972812418216753500));
         pointsBeforeIssue.add(new Point3D(-0.00600275773596248300, 0.06035648754052997000, -0.01130240420102690500));
         troublesomePoint.set(-0.82482261969415400000, -0.77419355574449570000, 2.14590750433259150000);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190317_161948 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190317_161948()
      {
         pointsBeforeIssue.add(new Point3D(0.01322660245066220600, 0.01363953726236516500, -0.00822103454813905000));
         pointsBeforeIssue.add(new Point3D(0.01358157291376119200, 0.01302508759426573800, -0.00864517544450316100));
         pointsBeforeIssue.add(new Point3D(0.01412701874723776300, 0.01241423690826731800, -0.00875830460347137900));
         troublesomePoint.set(0.01455927978213464300, 0.01304264340340427500, -0.00704666450967672550);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190321_222438 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190321_222438()
      {
         pointsBeforeIssue.add(new Point3D(-0.22221431798593744000, 0.20154601678536010000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.19164624264091773000, 0.23080666732488140000, 0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.22233983991481920000, 0.20140753607214537000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(0.01577966270187325400, -0.29958471630744965000, 0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.24694684726893160000, 0.17034451744607154000, 0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.26566620349137715000, -0.13936092824919830000, -0.50000000000000000000));

         troublesomePoint.set(-0.22089658280712426000, 0.20298940786192580000, -0.50000000000000000000);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_122756 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_122756()
      {
         pointsBeforeIssue.add(new Point3D(-0.27845244126420843000, 0.11164335158889922000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.27827933053495096000, 0.11207414597943410000, -0.50000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.27870324942520370000, 0.11101575906074180000, -0.50000000000000000000));

         troublesomePoint.set(0.24205618014166072000, -0.17722529631863476000, 0.50000000000000000000);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_124929 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_124929()
      {
         pointsBeforeIssue.add(new Point3D(0.04855499760929303000, -0.01193365858241087500, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02595809833327553000, -0.04273379378103467400, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(0.02574336606832163000, 0.04286349383184235000, -0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(0.04852589338486361400, -0.01205145929756432600, 0.15000000000000000000));
         troublesomePoint.set(0.04841411994552912600, -0.01249291758957528400, 0.15000000000000000000);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_150735 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_150735()
      {
         pointsBeforeIssue.add(new Point3D(-0.00127134727090134300, -0.04998383414781995000, -0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.03069378997379051300, -0.03947013120126202600, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02968907227182072000, -0.04023131849242088000, -0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02949883371674418000, -0.04037101447018488000, 0.15000000000000000000));
         pointsBeforeIssue.add(new Point3D(-0.02973125211791738700, -0.04020015730691651000, -0.15000000000000000000));
         troublesomePoint.set(-0.02889868575683430700, -0.04080276904240376000, -0.15000000000000000000);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_190624 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_190624()
      {
         pointsBeforeIssue.add(new Point3D(-0.02299062921644618000, -0.04440079918460806000, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.01961949482806400000, -0.04598994914860822500, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01954504118681128400, -0.04602164018161294000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01853556656231905000, -0.04643740703585678000, 0.001));
         troublesomePoint.set(-0.01764619662299450600, -0.04678260087620843000, -0.001);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_193234 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_193234()
      {
         pointsBeforeIssue.add(new Point3D(-0.01540314387381502300, -0.04756829993601363600, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.02299062921644618000, -0.04440079918460806000, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.01961949482806400000, -0.04598994914860822500, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01954504118681128400, -0.04602164018161294000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.01853556656231905000, -0.04643740703585678000, 0.001));
         troublesomePoint.set(-0.01764619662299450600, -0.04678260087620843000, -0.001);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_195449 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_195449()
      {
         pointsBeforeIssue.add(new Point3D(-0.03541014613115714400, -0.03530044689476462000, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.03734847754427659000, -0.03324291240437076600, 0.001));
         pointsBeforeIssue.add(new Point3D(-0.03534070522258475000, -0.03536996684152207000, -0.001));
         pointsBeforeIssue.add(new Point3D(-0.03543195159362232000, -0.03527856015014224600, -0.001));
         pointsBeforeIssue.add(new Point3D(0.01847795496318006800, 0.04646036138880852000, -0.001));
         troublesomePoint.set(-0.03581713708947079000, -0.03488742883495509600, -0.001);
         constructionEpsilon = 1.0E-6;
      }
   }

   public static class ConvexPolytope3DTroublesomeDataset_20190323_213507 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190323_213507()
      {
         pointsBeforeIssue.add(new Point3D(0.01569971687419740800, 0.04747124276938661400, -0.001));
         pointsBeforeIssue.add(new Point3D(0.00638643724321071700, 0.04959045693819055000,  0.001));
         pointsBeforeIssue.add(new Point3D(-0.04099078792044472000, 0.02863136925927790000, 0.001));
         pointsBeforeIssue.add(new Point3D(0.00562149156111026600, 0.04968298333059687000,  0.001));
         pointsBeforeIssue.add(new Point3D(0.00653865510674285100, 0.04957061618938245000,  0.001));
         pointsBeforeIssue.add(new Point3D(0.00542547626947362700, 0.04970477046772652600,  0.001));
                      troublesomePoint.set(0.00609110437244231200, 0.04962759764006328000,  0.001);
         constructionEpsilon = 1.0E-6;
      }
   }
}
