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
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.view.Window;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.GridLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.bumptech.glide.Glide;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;
import com.sothree.slidinguppanel.SlidingUpPanelLayout;

import android.speech.tts.TextToSpeech;
import androidx.appcompat.app.AppCompatActivity;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

public class RouteGuide extends AppCompatActivity {
    RecyclerView routeGuideRecyclerView;
    routeGuideAdapter odapter;
    static DatabaseReference databaseReference;
    List<Product> productList;
    Dialog dialog;
    private final String TTS_ID = "SenseMart_TTS";
    private TextToSpeech tts;
    ConstraintLayout topView;
    ImageView marker, marker1;

    int startX, endX, startY, endY; // 화면 뷰 크기 저장할 변수 (eX-sX, eY-sY)
    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.route_guidance);

        marker = findViewById(R.id.marker);
        marker1 = findViewById(R.id.marker1);
        topView = findViewById(R.id.topview);
        seeView(); //전체 뷰 크기 및 마커 함수들에서 뷰 크기를 알 수 있게 하기 위함!
        DestinationUpdate(); //도착 마커 생성
        MarkerUpdate(); // 로봇 위치 마커 생성
    }

    private void seeView() {
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
                                double endWidth = endX/449.0;
                                double endHeight = (endY-startY)/978.0;
                                if (coords.length == 2) {
                                    try {
                                        double x = -(Double.parseDouble(coords[1].trim()) - 11.123137474060059) / (0.05) * endWidth;
                                        double y = -(Double.parseDouble(coords[0].trim()) - 3.853449017) / (0.05) * endHeight;
//                                        Log.d("값값",endWidth+" "+endHeight);
                                        coordinateList.add(new double[]{x, y});

                                        marker.setX((float) x);
                                        marker.setY((float) y);
                                        Log.d("현재위치값", "현재X: " + x + "현재Y: " + y);
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
                            List<double[]> coordinateList1 = new ArrayList<>();

                            // 각 좌표 쌍을 파싱하여 리스트에 추가
                            for (String pair : coordinatePairs) {
                                String[] coords = pair.trim().split("\\s+");
                                double endWidth = endX/449.0;
                                double endHeight = (endY-startY)/978.0;
                                if (coords.length == 2) {
                                    try {
                                        double x = -(Double.parseDouble(coords[1].trim()) - 11.123137474060059) / (0.05) * endWidth;
                                        double y = -(Double.parseDouble(coords[0].trim()) - 3.853449017) / (0.05) * endHeight;
//                                        Log.d("값값",endWidth+" "+endHeight);
                                        coordinateList1.add(new double[]{x, y});

                                        marker.setX((float) x);
                                        marker.setY((float) y);
                                        Log.d("목적값", "목적X: " + x + "목적Y: " + y);
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
        tts.speak("장바구니에 상품이 없습니다. 경로 안내를 종료합니다.", TextToSpeech.QUEUE_FLUSH, null, TTS_ID);
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
}

