package com.example.sensemart;

import static android.net.wifi.p2p.WifiP2pManager.ERROR;

import android.app.Dialog;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Bundle;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.view.Window;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.algolia.search.saas.AlgoliaException;
import com.algolia.search.saas.Client;
import com.algolia.search.saas.CompletionHandler;
import com.algolia.search.saas.Index;
import com.algolia.search.saas.Query;
import com.bumptech.glide.Glide;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;
import com.sothree.slidinguppanel.SlidingUpPanelLayout;

import android.speech.tts.TextToSpeech;
import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

public class RouteGuide extends AppCompatActivity {

    RecyclerView routeGuideRecyclerView;
    routeGuideAdapter odapter;
    static DatabaseReference databaseReference;
    List<Product> productList;
    Dialog dialog;
    ImageView marker,marker1;
    ConstraintLayout topView;
    TextView guide_text;
    private final String TTS_ID = "SenseMart_TTS";
    private TextToSpeech tts;
    private Client client; // Algolia 검색 client
    int startX, endX, startY, endY; // 화면 뷰 크기 저장할 변수 (eX-sX, eY-sY)
    double bx, by;
    Map<String, String> waypointMap;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.route_guidance);
        client = new Client("appID", "key"); // Algolia 클라이언트 초기화

        marker = findViewById(R.id.marker);
        marker1 = findViewById(R.id.marker1);
        topView = findViewById(R.id.topview);
        guide_text = findViewById(R.id.guide_text);

        routeGuideRecyclerView = findViewById(R.id.route_list);
        routeGuideRecyclerView.setLayoutManager(new LinearLayoutManager(this));

        odapter = new routeGuideAdapter(this);
        routeGuideRecyclerView.setAdapter(odapter);

        productList = new ArrayList<>();
        databaseReference = FirebaseDatabase.getInstance().getReference("products");

        // 장바구니에 상품이 없을 때 안내하기 위한 Dialog
        dialog = new Dialog(this);
        dialog.requestWindowFeature(Window.FEATURE_NO_TITLE);
        dialog.setContentView(R.layout.route_dialog);
        Objects.requireNonNull(dialog.getWindow()).setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));

        initTextToSpeech();

        getProducts(); // 경로 안내할 상품 불러오는 함수
        seeView(); //전체 뷰 크기 및 마커 함수들에서 뷰 크기를 알 수 있게 하기 위함!
        DestinationUpdate(); //도착 마커 생성
        MarkerUpdate(); // 로봇 위치 마커 생성
        ttsGuid(); //경유지에 도착했는지 확인하는 함수

        waypointMap = new HashMap<>();
        waypointMap.put("7.175329208374023 1.1965241432189941", "banana");
        waypointMap.put("-6.476901531219482 5.273036479949951", "homerunball");
        waypointMap.put("7.175329208374023 -1.0462605953216553", "nobrandmilk");
        waypointMap.put("-6.449102878570557 3.969860076904297", "yeahgam");
    }
    private void seeView () {
        // 뷰의 위치와 크기 정보를 얻는 리스너 추가
        topView.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                int[] location = new int[2];
                topView.getLocationOnScreen(location);

                startX = location[0]; // x 좌표
                startY = location[1]; // y 좌표
                int width = topView.getWidth(); // 뷰의 너비
                int height = topView.getHeight(); // 뷰의 높이

                // 총 끝 x, y 좌표 계산
                endX = startX + width; // 끝 x 좌표
                endY = startY + height; // 끝 y 좌표

                Log.d("Coordinates", "Start X: " + startX + ", End X: " + endX + ", Start Y: " + startY + ", End Y: " + endY);

                // 리스너 해제 (한 번만 실행되도록)
                topView.getViewTreeObserver().removeOnGlobalLayoutListener(this);
            }
        });
    }
    private void MarkerUpdate() {
        Handler updateHandler = new Handler();
        DatabaseReference markerDB = FirebaseDatabase.getInstance().getReference("admin/marker/base_link");
        Runnable updateRunnable = new Runnable() {
            @Override
            public void run() {
                markerDB.addListenerForSingleValueEvent(new ValueEventListener() {
                    @Override
                    public void onDataChange(@NonNull DataSnapshot snapshot) {
                        String coordinates = snapshot.getValue(String.class); // 좌표가 담긴 경로의 리스트를 모두 불러옴
                        if (coordinates != null) { // 그 값이 존재할 때
                            String[] coordinatePairs = coordinates.replace("[", "").replace("]", "").trim().split(",\\s*");
                            List<double[]> coordinateList = new ArrayList<>();

                            // 각 좌표 쌍을 파싱하여 리스트에 추가
                            for (String pair : coordinatePairs) {
                                String[] coords = pair.trim().split("\\s+");
                                double endWidth = (endX-startX)/449.0;
                                double endHeight = (endY-startY)/978.0+0.05;
                                if (coords.length == 2) {
                                    try {
                                        bx = -(Double.parseDouble(coords[1].trim()) - 11.000137474060059) / (0.05) * endWidth;
                                        by = -(Double.parseDouble(coords[0].trim()) - 3.853449017) / (0.05) * endHeight;
                                        Log.d("값값",endWidth+" "+endHeight);
                                        coordinateList.add(new double[]{bx, by});

                                        marker.setX((float) bx);
                                        marker.setY((float) by);
                                        Log.d("현재위치값", "현재X: " + bx + "현재Y: " + by);
                                    } catch (NumberFormatException e) {
                                        Log.e("ParseError", "Failed to parse coordinate", e);
                                    }
                                }
                            }
                        }
                    }

                    @Override
                    public void onCancelled(@NonNull DatabaseError error) {
                        Log.e("DatabaseError", "Error reading marker coordinates: " + error.getMessage());
                    }
                });
                updateHandler.postDelayed(this, 500);
            }
        };

        updateHandler.post(updateRunnable);
    }
    private void DestinationUpdate() {
        Handler updateHandler = new Handler();
        DatabaseReference markerDB = FirebaseDatabase.getInstance().getReference("admin/marker/waypoints");
        Runnable updateRunnable = new Runnable() {
            @Override
            public void run() {
                markerDB.addListenerForSingleValueEvent(new ValueEventListener() {
                    @Override
                    public void onDataChange(@NonNull DataSnapshot snapshot) {
                        String coordinates = snapshot.getValue(String.class); // 좌표가 담긴 경로의 리스트를 모두 불러옴
                        if (coordinates != null) { // 그 값이 존재할 때
                            String[] coordinatePairs = coordinates.replace("[", "").replace("]", "").trim().split(",\\s*");
                            List<double[]> coordinateList1 = new ArrayList<>();
                            List<Double> distanceDifferences = new ArrayList<>(); // 거리 차이를 저장

                            // 각 좌표 쌍을 파싱하여 리스트에 추가
                            for (String pair : coordinatePairs) {
                                String[] coords = pair.trim().split("\\s+");
                                double endWidth = (endX - startX) / 449.0;
                                double endHeight = (endY - startY) / 978.0;
                                double coordiX = Double.parseDouble(coords[1].trim()); //기존 Db에 저장된 X좌표 값
                                if (coords.length == 2) {
                                    try {
                                        double x = -(coordiX - (11.000137474060059)) / (0.05) * endWidth;
                                        double y = (Double.parseDouble(coords[0].trim()) + 3.853449017) / (0.05) * endHeight;
                                        Log.d("목적좌표", (Double.parseDouble(coords[1].trim()) + "  " + (Double.parseDouble(coords[0].trim()))));

//                                        Log.d("값값",endWidth+" "+endHeight);

                                        if (coordiX == 1.1965241432189941) { //왼쪽 마커들
                                            x -= 25;
                                        } else if (coordiX == -1.0462605953216553) { //오른쪽 마커들
                                            x += 25;
                                        }
                                        coordinateList1.add(new double[]{x, y});

                                        // 거리 차이 계산
                                        double distanceDifference = Math.sqrt(Math.pow(bx - x, 2) + Math.pow(by - y, 2));
                                        distanceDifferences.add(distanceDifference);

//                                        marker1.setX((float) x);
//                                        marker1.setY((float) y);
                                        Log.d("목적값", "목적X: " + x + "목적Y: " + y);
//                                        Log.d("거리 차이", (bx - x) + " " + (by - y));

                                    } catch (NumberFormatException e) {
                                        Log.e("ParseError", "Failed to parse coordinate", e);
                                    }
                                }
                            }
                            // 가장 작은 거리 차이를 가진 인덱스 찾기
                            if (!distanceDifferences.isEmpty()) {
                                double minDistance = Double.MAX_VALUE;
                                int minIndex = -1;

                                for (int i = 0; i < distanceDifferences.size(); i++) {
                                    if (distanceDifferences.get(i) < minDistance) {
                                        minDistance = distanceDifferences.get(i);
                                        minIndex = i;
                                    }
                                }

                                // 가장 작은 거리 차이를 가진 좌표로 marker1 위치 설정
                                if (minIndex != -1) {
                                    double[] bestCoordinate = coordinateList1.get(minIndex);
                                    marker1.setX((float) bestCoordinate[0]);
                                    marker1.setY((float) bestCoordinate[1]);
                                    Log.d("최적값", "최적X: " + bestCoordinate[0] + " 최적Y: " + bestCoordinate[1]);
                                }
                            }
                        }
                    }


                    @Override
                    public void onCancelled(@NonNull DatabaseError error) {
                        Log.e("DatabaseError", "Error reading marker coordinates: " + error.getMessage());
                    }
                });
                updateHandler.postDelayed(this, 550);
            }
        };

        updateHandler.post(updateRunnable);
    }
    public void removeWaypoints(String coordinate) {
        DatabaseReference waypointsRef = FirebaseDatabase.getInstance().getReference("admin/marker/waypoints");

        waypointsRef.addListenerForSingleValueEvent(new ValueEventListener() {
            @Override
            public void onDataChange(DataSnapshot dataSnapshot) {
                List<String> currentWaypoints = new ArrayList<>(); // currentWaypoints 초기화

                // 현재 저장된 데이터를 가져오기
                if (dataSnapshot.exists()) {
                    String currentData = dataSnapshot.getValue(String.class);
                    if (currentData != null && !currentData.isEmpty()) {
                        // 괄호 제거 및 리스트로 변환
                        currentWaypoints = new ArrayList<>(Arrays.asList(currentData.replace("[", "").replace("]", "").split(", ")));
                    }
                }

                // 좌표가 존재하면 삭제
                if (currentWaypoints.contains(coordinate)) {
                    currentWaypoints.remove(coordinate); // 좌표 삭제

                    // 삭제된 좌표가 있을 경우 전체 데이터를 문자열로 변환
                    String finalWaypointsString;

                    if (currentWaypoints.isEmpty()) {
                        // 웨이포인트가 없으면 대괄호만 포함한 문자열로 설정
                        finalWaypointsString = "[]";
                    } else {
                        // List의 toString() 메서드 사용하여 문자열로 변환
                        finalWaypointsString = currentWaypoints.toString(); // 대괄호는 toString() 메서드에서 자동 추가됨
                    }

                    // Firebase에 설정
                    waypointsRef.setValue(finalWaypointsString);
                    Log.d("삭제", "좌표값 삭제: " + coordinate); // 로그 추가
                } else {
                    Log.d("삭제", "삭제할 좌표가 존재하지 않습니다: " + coordinate);
                }
            }

            @Override
            public void onCancelled(DatabaseError databaseError) {
                System.err.println("Failed to read data: " + databaseError.getMessage());
            }
        });
    }
    private void initTextToSpeech() {
        if (tts == null) {
            tts = new TextToSpeech(this, new TextToSpeech.OnInitListener() {
                @Override
                public void onInit(int status) {
                    if (status == TextToSpeech.SUCCESS) {
                        int result = tts.setLanguage(Locale.KOREAN);
                        if (result == TextToSpeech.LANG_MISSING_DATA || result == TextToSpeech.LANG_NOT_SUPPORTED) {
                            Log.e("TTS", "Language is not supported");
                        } else {
                            if (productList.isEmpty()){
                                speakText("장바구니에 상품이 없습니다. 경로 안내를 종료합니다.");
                            }
                            else{
                                ttsGuid();
                            }
                        }
                    } else {
                        Log.e("TTS", "Initialization failed");
                    }
                }
            });
        }
    }

    // 여기에 TTS할 것들 추가
    private  void ttsGuid() {
        DatabaseReference arrivedDB = FirebaseDatabase.getInstance().getReference("admin");

        arrivedDB.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {
                String arrived = snapshot.child("marker").child("arrived").getValue(String.class);
                String waypointsStr = snapshot.child("marker").child("waypoints").getValue(String.class);

                arrived = arrived.replace("[", "").replace("]", "").trim(); // 대괄호와 공백 제거

                // 대괄호 없이 List에 저장
                List<String> waypointsList = new ArrayList<>();
                if (waypointsStr != null) {
                    waypointsStr = waypointsStr.replace("[", "").replace("]", "").trim();
                    String[] waypoints = waypointsStr.split(",\\s*");
                    for (String waypoint : waypoints) {
                        waypointsList.add(waypoint.trim());
                    }
                }

                // 도착 좌표와 비교
                if (arrived != null && !waypointsList.isEmpty()) {
                    for (String waypoint : waypointsList) {
                        Log.d("비교", "현재 waypoint: " + waypoint);
                        Log.d("비교하는 애", "비교당하는 애: " + arrived);
                        if (waypoint.equals(arrived)) {
                            Log.d("비교결과", "일치하는 좌표 발견: " + waypoint);
                            Log.d("비교결과", "일치하는 좌표 발견: " + waypointMap.get(waypoint));

                            // 알고리아에서 objectID를 가져오는 부분
                            Query query = new Query(""); // 모든 데이터를 검색
                            Index index = client.getIndex("basket");
                            query.setHitsPerPage(100); // 한 번에 가져올 데이터 수 설정

                            // 비동기 검색 실행
                            index.searchAsync(query, new CompletionHandler() {
                                @Override
                                public void requestCompleted(JSONObject content, AlgoliaException error) {
                                    if (error != null) {
                                        Log.e("Algolia", "Search error", error);
                                        return;
                                    }
                                    try {
                                        // 'hits' 배열을 가져와서 처리
                                        JSONArray hits = content.getJSONArray("hits");
                                        for (int i = 0; i < hits.length(); i++) {
                                            JSONObject json = hits.getJSONObject(i); // 각 검색 결과 항목을 JSONObject로 처리

                                            // objectID 가져오기
                                            String objectId = json.getString("objectID"); // 'objectID' 필드에서 가져오기
                                            String productName = json.getString("productName");
                                            Log.d("찾은 objectID", objectId);

                                            // 추가적인 처리 로직
                                            // 예: objectId와 arrived 비교
                                            if (objectId.equals(waypointMap.get(waypoint))) {
                                                Log.d("비교결과", "일치하는 objectID 발견: " + objectId);
                                                Log.d("비교결과","일치하는 상품명은?: "+productName);
                                                guide_text.setText(productName+"에 도착했습니다");
                                                speakText(productName+"에 도착했습니다.");
                                                Integer bracelet = snapshot.child("marker").child("bracelet").getValue(Integer.class);
                                                if (bracelet != null && bracelet == 1) { //팔찌랑 해당하는 상품이 겹칠 때!
                                                    speakText(productName+"이 맞습니다.");
                                                    String coordinate = json.getString("coordinate");
                                                    removeWaypoints(coordinate);
                                                    DatabaseReference productRef =FirebaseDatabase.getInstance().getReference("products").child(productName);
                                                    productRef.removeValue(); // DB 상에서 삭제
                                                }
                                                else{
                                                    speakText(productName+"이 아닙니다.");
                                                }

                                            }
                                        }
                                    } catch (JSONException ex) {
                                        Log.e("JSON 처리 에러", "Error processing JSON: " + ex.getMessage());
                                    }
                                }
                            });
                            break; // 일치하는 waypoint를 찾았으므로 루프 종료
                        } else {
                            Log.d("없음", "없다");
                        }
                    }
                }
            }

            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                Log.e("DatabaseError", "Error reading arrived status: " + error.getMessage());
            }
        });
    }

    private void speakText(String text) {
        if (tts != null) {
            tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, null);
        } else {
            Log.e("TTS", "TTS is not initialized");
        }
    }

    /* 경로 안내할 상품 리스트 불러오기 */
    private void getProducts() {
        databaseReference.addValueEventListener(new ValueEventListener() { // "products" 경로에 대한 ValueEventListener을 추가하여 DB 값을 읽고 변화에 대한 알림을 받도록 함
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) { // DB에서 데이터가 변경될 때마다 호출 → 데이터의 변경 사항을 실시간으로 수신하기 위함
                productList.clear(); // 중복된 리스트가 추가되지 않기 위해 productList 초기화
                for (DataSnapshot dataSnapshot : snapshot.getChildren()) {
                    // 'snapshot'은 "products" 아래에 있는 모든 데이터
                    // for문이 반복할 때마다 'snapshot.getChildren()'이 반환한 자식 데이터를 'dataSnapshot' 변수에 하나씩 대입하여 반복
                    Product product = dataSnapshot.getValue(Product.class); // 'Product' 클래스에 정의된 필드와 일치하는 속성들을 "제품 name" 데이터에서 가져와서 이를 Product 객체로 변환 (이름,가격,위치)
                    if (product != null) {  // 'product' 객체가 null이 아닌지 확인 → null인 경후 데이터 변환 실패
                        productList.add(product); // 'product' 객체를 'productList"에 추가
                    }
                }
                odapter.updateProducts(productList); // Adapter에 'productList'를 전달하여 RecyclerView 갱신
                setProductNames(); // DB로부터 불러온 후 상단 경로 안내 이름 설정 -> DB가 다 불러오기 전에 함수를 불러오게 설정하면 오류가 생길 수 있음
            }

            @Override // 로그에 오류 메시지 출력
            public void onCancelled(@NonNull DatabaseError error) {
                Log.e("RouteGuide", "Database error: " + error.getMessage());
            }
        });

    }

    /* 상단에 (예감 > 콜라) 같은 안내를 하기 위한 이름 설정*/
    private void setProductNames() {
        TextView first_product = findViewById(R.id.first_product);
        TextView second_product = findViewById(R.id.second_product);
        LinearLayout route_top = findViewById(R.id.top_route);

        if (productList.isEmpty()){
            route_top.setVisibility(View.GONE);
            showDialog();
        }

        if (!productList.isEmpty()) {
            // 첫 번째 제품 이름 설정
            first_product.setText(productList.get(0).getName());

            // 두 번째 제품이 있는 경우에만 설정
            if (productList.size() > 1) {
                second_product.setText(productList.get(1).getName());
            } else {
                second_product.setText("안내 종료");
            }
        }
    }

    /* '장바구니에 상품이 없습니다. 경로 안내를 종료합니다.' 안내 Dialog */
    private void showDialog() {
        Button yes_btn = dialog.findViewById(R.id.yes_btn);
        yes_btn.setOnClickListener(v -> finish());
        dialog.show();
    }


    public static class routeGuideAdapter extends RecyclerView.Adapter<routeGuideAdapter.ViewHolder> {

        private final List<Product> productList;
        private final LayoutInflater inflater;

        public routeGuideAdapter(Context context) {
            this.productList = new ArrayList<>();
            this.inflater = LayoutInflater.from(context);
        }

        @NonNull
        @Override
        public routeGuideAdapter.ViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            View view = inflater.inflate(R.layout.route_guide_frame, parent, false);
            return new ViewHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull routeGuideAdapter.ViewHolder holder, int position) {
            Product product = productList.get(position);
            holder.productName.setText(product.getName()); // ViewHolder 내의 TextView인 'productName'에 'product.getName()'을 이용하여 text 지정
            holder.productLocation.setText(product.getLocation());
            holder.productPrice.setText(product.getPrice());

            Glide.with(holder.itemView.getContext())
                    .load(product.getImageUrl())
                    .into(holder.productImage);
        }

        @Override
        public int getItemCount() {return productList.size();}
        public void updateProducts(List<Product> products) {
            this.productList.clear();
            this.productList.addAll(products);
            notifyDataSetChanged();
        }
        public static class ViewHolder extends RecyclerView.ViewHolder {

            ImageView productImage;
            TextView productName;
            TextView productLocation;
            TextView productPrice;

            public ViewHolder( View view) {
                super(view);
                productImage = view.findViewById(R.id.product_img);
                productName = view.findViewById(R.id.product_name);
                productLocation = view.findViewById(R.id.product_location);
                productPrice = view.findViewById(R.id.product_price);
            }
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        // 액티비티 일시 정지 시 음성 정지
        if (tts != null && tts.isSpeaking()) {
            tts.stop();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // 액티비티 종료 시 TTS 자원 해제
        if (tts != null) {
            tts.stop();
            tts.shutdown();
            tts = null;
        }
    }

}