package com.example.sensemart;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.Fragment;

import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.util.Log;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.recyclerview.widget.RecyclerView;

import com.algolia.search.saas.Client;
import com.example.sensemart.models.Message;
import com.example.sensemart.helpers.SendMessageInBg;
import com.example.sensemart.interfaces.BotReply;
import com.google.android.material.snackbar.Snackbar;
import com.google.api.gax.core.FixedCredentialsProvider;
import com.google.auth.oauth2.GoogleCredentials;
import com.google.auth.oauth2.ServiceAccountCredentials;
import com.google.cloud.dialogflow.v2.DetectIntentResponse;
import com.google.cloud.dialogflow.v2.QueryInput;
import com.google.cloud.dialogflow.v2.SessionName;
import com.google.cloud.dialogflow.v2.SessionsClient;
import com.google.cloud.dialogflow.v2.SessionsSettings;
import com.google.cloud.dialogflow.v2.TextInput;
import com.google.common.collect.Lists;


import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

import com.algolia.search.saas.Client;
import com.algolia.search.saas.Index;
import com.algolia.search.saas.Query;
import com.algolia.search.saas.CompletionHandler;
import com.algolia.search.saas.AlgoliaException;
import com.bumptech.glide.Glide;
import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.UUID;

public class MainMenuAiFragment extends Fragment implements BotReply {

    RecyclerView chatView;
    private ChatAdapter chatAdapter;
    List<Message> messageList = new ArrayList<>();
    EditText editMessage;
    ImageButton btnSend, sttButton, trashButton;
    String modifyBotReply, sttText;
    int completedRequests;
    private Client client; // Algolia 검색 client
    String productName, productLocation, productPrice, productImage, coordinate= null;
    private ArrayList<String> indices;
    // Dialogflow
    private SessionsClient sessionsClient;
    private SessionName sessionName;
    private String uuid = UUID.randomUUID().toString();
    private String TAG = "MainMenuAiFragment";

    final int PERMISSION = 1;
    String[] REQUIRED_PERMISSION = {Manifest.permission.INTERNET, Manifest.permission.RECORD_AUDIO};
    Intent intent;

    private View mLayout; // snackbar 사용

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.chatbot, container, false);
        client = new Client("3IZY9Y1G61", "3630c5bf5e6b7c91de409f8f364983b1"); // Algolia 클라이언트 초기화

        // 리스트에 제품 카테고리 인덱스 추가
        indices = new ArrayList<>();

        /* 가공식품 */
        indices.add("snack_dessert");
        indices.add("healthyfood");
        indices.add("noodles_canned");
        indices.add("mealkits_simplefood");
        indices.add("sidedishes_deli");
        indices.add("bakery_jam");
        indices.add("seasoning_oil");
        indices.add("milk_dailyproducts");
        indices.add("eco_friendly_organic");
        indices.add("drink_liquor");
        indices.add("rice_multi_grain");
        indices.add("coffee_tea");

        /* 신선식품 */
        indices.add("fruit");
        indices.add("vegetable");
        indices.add("meats_eggs");

        /* 화장품 */
        indices.add("hair");
        indices.add("body");
        indices.add("beauty");

        /* 스포츠 */
        indices.add("car");
        indices.add("trip");
        indices.add("workout");

        /* 언더웨어 */
        indices.add("boyfashion");
        indices.add("girlfashion");
        indices.add("unisex");

        /* 생활 */
        indices.add("childeren_toys");
        indices.add("cleaning_housegoods");
        indices.add("dailygoods");
        indices.add("digital_home_appliances");
        indices.add("furniture_interior");
        indices.add("hobby_book");
        indices.add("hygiene_health");
        indices.add("kitchenware");
        indices.add("miscellaneous_luxury_goods");
        indices.add("pet");

        chatView = rootView.findViewById(R.id.chatView);
        editMessage = rootView.findViewById(R.id.editMessage);
        btnSend = rootView.findViewById(R.id.btnSend);
        sttButton = rootView.findViewById(R.id.mic_btn);
        trashButton = rootView.findViewById(R.id.trashbutton);

        chatAdapter = new ChatAdapter(messageList, this);
        chatView.setAdapter(chatAdapter);

        /* 퍼미션 체크 및 권한 요청, 음성 인식 사용자 설정*/
        if (Build.VERSION.SDK_INT >= 23) {
            // 권한 요청
            ActivityCompat.requestPermissions(requireActivity(), REQUIRED_PERMISSION, PERMISSION);
        }

        intent=new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,getActivity().getPackageName());
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE,"ko-KR");   // 텍스트로 변환시킬 언어 설정

        /* 음성 인식 버튼 이벤트 */
        sttButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                SpeechRecognizer mRecognizer = SpeechRecognizer.createSpeechRecognizer(requireActivity());
                mRecognizer.setRecognitionListener(listener);

                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        // 버튼을 눌렀을 때 음성 인식 시작
                        mRecognizer.startListening(intent);
                        break;
                    case MotionEvent.ACTION_UP:
                        // 버튼에서 손을 뗐을 때 음성 인식 종료
                        mRecognizer.stopListening();
                        break;
                }
                return true;
            }
        });

        /* 메세지 보내기 버튼 이벤트*/
        btnSend.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String message = editMessage.getText().toString();
                if (!message.isEmpty()) {
                    messageList.add(new Message(message, false));
                    editMessage.setText("");
                    sendMessageToBot(message); // 입력된 메시지를 sendMessageToBot() 함수를 통해 처리
                    Objects.requireNonNull(chatView.getAdapter()).notifyDataSetChanged();
                    Objects.requireNonNull(chatView.getLayoutManager()).scrollToPosition(messageList.size() - 1);
                } else {
                    Toast.makeText(getContext(), "Please enter text!", Toast.LENGTH_SHORT).show();
                }
            }
        });

        trashButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // 메시지 리스트를 빈 리스트로 초기화
                messageList.clear();
                // 어댑터에 데이터 변경을 알리고 뷰 갱신
                Objects.requireNonNull(chatView.getAdapter()).notifyDataSetChanged();
            }
        });

        setUpBot();

        return rootView;
    }

    // 권한 요청 결과 처리 (사용자가 권한을 허용했는지 여부를 확인)
    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == 1) {
            for (int i = 0; i < permissions.length; i++) {
                if (grantResults[i] != PackageManager.PERMISSION_GRANTED) {
                    Snackbar.make(mLayout, "이 기능을 실행하려면 마이크 접근 권한이 필요합니다.", Snackbar.LENGTH_INDEFINITE);
                }
            }
        }
    }
    private RecognitionListener listener = new RecognitionListener() {
        @Override
        public void onReadyForSpeech(Bundle params) {
            Toast.makeText(requireActivity(),"음성인식을 시작합니다.",Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onBeginningOfSpeech() {}

        @Override
        public void onRmsChanged(float rmsdB) {}

        @Override
        public void onBufferReceived(byte[] buffer) {}

        @Override
        public void onEndOfSpeech() {}

        @Override
        public void onError(int error) {
            String message;

            switch (error) {
                case SpeechRecognizer.ERROR_AUDIO:
                    message = "오디오 에러";
                    break;
                case SpeechRecognizer.ERROR_CLIENT:
                    message = "클라이언트 에러";
                    break;
                case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                    message = "퍼미션 없음";
                    break;
                case SpeechRecognizer.ERROR_NETWORK:
                    message = "네트워크 에러";
                    break;
                case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                    message = "네트웍 타임아웃";
                    break;
                case SpeechRecognizer.ERROR_NO_MATCH:
                    message = "찾을 수 없음";
                    break;
                case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                    message = "RECOGNIZER가 바쁨";
                    break;
                case SpeechRecognizer.ERROR_SERVER:
                    message = "서버가 이상함";
                    break;
                case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                    message = "말하는 시간초과";
                    break;
                default:
                    message = "알 수 없는 오류임";
                    break;
            }
            Toast.makeText(requireActivity(), "에러가 발생하였습니다. : " + message,Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onResults(Bundle results) {
            ArrayList<String> matches =
                    results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);

            for(int i = 0; i < matches.size(); i++){
                sttText = (matches.get(i));
                if (!sttText.isEmpty()) {
                    messageList.add(new Message(sttText, false));
                    sendMessageToBot(sttText); // 입력된 메시지를 sendMessageToBot() 함수를 통해 처리
                    Objects.requireNonNull(chatView.getAdapter()).notifyDataSetChanged();
                    Objects.requireNonNull(chatView.getLayoutManager()).scrollToPosition(messageList.size() - 1);
                } else {
                    Toast.makeText(getContext(), "음성이 인식되지 않았습니다.", Toast.LENGTH_SHORT).show();
                }
            }

        }

        @Override
        public void onPartialResults(Bundle partialResults) {}

        @Override
        public void onEvent(int eventType, Bundle params) {}
    };


    /* 챗봇 client 및 id 세팅*/
    private void setUpBot() {
        try {
            InputStream stream = getResources().openRawResource(R.raw.credential);
            GoogleCredentials credentials = GoogleCredentials.fromStream(stream)
                    .createScoped(Lists.newArrayList("https://www.googleapis.com/auth/cloud-platform"));
            String projectId = ((ServiceAccountCredentials) credentials).getProjectId();

            SessionsSettings.Builder settingsBuilder = SessionsSettings.newBuilder();
            SessionsSettings sessionsSettings = settingsBuilder.setCredentialsProvider(
                    FixedCredentialsProvider.create(credentials)).build();
            sessionsClient = SessionsClient.create(sessionsSettings);
            sessionName = SessionName.of(projectId, uuid);

            Log.d(TAG, "projectId : " + projectId);
        } catch (Exception e) {
            Log.d(TAG, "setUpBot: " + e.getMessage());
        }
    }

    /* 메시지 처리 함수 */
    private void sendMessageToBot(String message) {
        QueryInput input = QueryInput.newBuilder()
                .setText(TextInput.newBuilder().setText(message).setLanguageCode("en-US")).build();
        new SendMessageInBg(this, sessionName, sessionsClient, input).execute();
    }

    /* 메시지에 대한 chatbot의 답변 처리 함수*/
    @Override
    public void callback(DetectIntentResponse returnResponse) {
        if (returnResponse != null) {
            String botReply = returnResponse.getQueryResult().getFulfillmentText(); // chatbot 답변
            if (!botReply.isEmpty()) {
                switch (botReply) {
                    case "오류":
                        fallback();
                        break;
                    case "장바구니":
                        productInBaket();
                        break;
                    case "경로안내":
                        productRouteGuide();
                        break;
                    case "인사":
                        modifyBotReply = "안녕하세요!";
                        sendMessageBot(modifyBotReply);
                        break;
                    default:
                        searchBotReply(botReply);
                        break;
                }
            } else {
                Toast.makeText(getContext(), "Something went wrong", Toast.LENGTH_SHORT).show();
            }
        } else {
            Toast.makeText(getContext(), "Failed to connect!", Toast.LENGTH_SHORT).show();
        }
    }

    /* 데이터 처리 완료 후 되묻는 함수*/
    private void request(){
        modifyBotReply = "더 필요한 것이 있으신가요?";
        sendMessageBot(modifyBotReply);
    }

    /* 오류 발생 시 처리 함수 */
    private void fallback() {
        modifyBotReply = "다시 검색해주세요.";
        sendMessageBot(modifyBotReply);
    }

    /* 2번 경로 안내받기 선택시 함수*/
    private void productRouteGuide() {
        if (productName != null && productLocation != null && productPrice != null && productImage != null){
            if (productName.equals("예감") || productName.equals("홈런볼")
                    || productName.equals("노브랜드 굿모닝 굿밀크 1L") || productName.equals("dole 필리핀산 바나나 1200g")
                    || productName.equals("케라시스 러블리 앤 로맨틱 퍼품 샴푸") || productName.equals("다온 피크닉 캠핑 돗자리")
                    || productName.equals("윈드 브레이커") || productName.equals("로지텍 무선 마우스 B175")) {

                modifyBotReply = productName + " 경로 안내를 시작합니다.";
                sendMessageBot(modifyBotReply);

                DatabaseReference databaseReference = FirebaseDatabase.getInstance().getReference("products");
                databaseReference.removeValue().addOnCompleteListener(new OnCompleteListener<Void>() {
                    @Override
                    public void onComplete(@NonNull Task<Void> task) {
                        if (task.isSuccessful()) {
                            // 삭제 성공 후 새로운 상품 추가
                            addProductToBasket(databaseReference, productName, productLocation, productPrice, productImage, coordinate);

                            // 모든 처리 완료 후 RouteGuide 액티비티 시작
                            Intent intent = new Intent(getActivity(), RouteGuide.class);
                            startActivity(intent);
                        } else {
                            // 삭제 실패
                            Log.e("Firebase", "Failed to delete products: " + task.getException().getMessage());
                        }
                    }
                });
            }
            else {
                modifyBotReply = "상품 준비중입니다.";
                sendMessageBot(modifyBotReply);
            }
        }
        else {
            modifyBotReply = "경로 안내할 상품이 없습니다.";
            sendMessageBot(modifyBotReply);
        }
    }

    /* 1번 장바구니 담기 선택시 함수*/
    private void productInBaket() {
        if (productName != null && productLocation != null && productPrice != null && productImage != null){
            if (productName.equals("예감") || productName.equals("홈런볼")
                    || productName.equals("노브랜드 굿모닝 굿밀크 1L") || productName.equals("dole 필리핀산 바나나 1200g")
                    || productName.equals("케라시스 러블리 앤 로맨틱 퍼품 샴푸") || productName.equals("다온 피크닉 캠핑 돗자리")
                    || productName.equals("윈드 브레이커") || productName.equals("로지텍 무선 마우스 B175")) {
                DatabaseReference databaseReference = FirebaseDatabase.getInstance().getReference("products");
                addProductToBasket(databaseReference, productName, productLocation, productPrice, productImage,coordinate);

                modifyBotReply = productName + "가 장바구니에 담겼습니다.";
                sendMessageBot(modifyBotReply);
                request();
            }
            else {
                modifyBotReply = "상품 준비중입니다.";
                sendMessageBot(modifyBotReply);
            }
        }
        else {
            modifyBotReply = "장바구니에 담을 상품이 없습니다.";
            sendMessageBot(modifyBotReply);
            request();
        }
    }

    /* Realtime DB에 장바구니에 담을 상품 처리*/
    private void addProductToBasket(DatabaseReference databaseReference, String name, String location, String price, String imageUrl,String coordinate) {
        DatabaseReference productsRef = databaseReference.child(name);
        Product product = new Product(name, price, location, imageUrl,coordinate);
        productsRef.setValue(product);
    }

    /* 상품 검색시 함수*/
    private void searchBotReply(String botReply) {
        completedRequests = 0;
        final int totalRequests = indices.size();
        modifyBotReply = null;

        for (String indexName : indices) {
            Index index = client.getIndex(indexName);

            // botReply에 대한 Algolia 검색 수행
            index.searchAsync(new Query(botReply).setHitsPerPage(100), new CompletionHandler() {
                @Override
                public void requestCompleted(JSONObject content, AlgoliaException error) {
                    if (error != null) {
                        Log.e("Algolia", "Search error", error);
                        return;
                    }
                    try {
                        JSONArray hits = content.getJSONArray("hits");

                        for (int i = 0; i < hits.length(); i++) {
                            JSONObject json = hits.getJSONObject(i);
                            String name = json.optString("productName");
                            String location = json.optString("productLocation");
                            String price = json.optString("productPrice");
                            String imageUrl = json.optString("url", "");

                            if (name.equalsIgnoreCase(botReply)) {  // botReply와 일치하는지 확인후 일치하면 변수에 저장
                                modifyBotReply = botReply + "는 " + location + "에 있습니다. \n 1번 장바구니에 담기 \n 2번 경로 안내받기";
                                productName = name;
                                productLocation = location;
                                productPrice = price;
                                productImage = imageUrl;
                                break; // 검색 중지
                            }
                            else { // botReply와 일치하는 상품이 없을 시 null값 저장
                                modifyBotReply = null;
                            }
                        }
                    } catch (JSONException e) {
                        Log.e("Algolia", "JSON Parsing error", e);
                    }

                    completedRequests++;

                    // 동기화를 위해 모든 검색이 완료된 후 Message 처리
                    if (completedRequests == totalRequests) {
                        sendMessageBot(modifyBotReply);
                    }
                }
            });
        }
    }

    private void sendMessageBot(String modifyBotReply){
        if (modifyBotReply != null) {
            messageList.add(new Message(modifyBotReply, true));
            chatAdapter.notifyDataSetChanged();
            Objects.requireNonNull(chatView.getLayoutManager()).scrollToPosition(messageList.size() - 1);
        }
        else { // null인 경우 (상품 검색 결과가 없거나 실패했을 시 오류 처리 함수로 이동)
            fallback();
        }
    }

    public class ChatAdapter extends RecyclerView.Adapter<ChatAdapter.MyViewHolder> {

        private List<Message> messageList;
        private Fragment fragment;

        public ChatAdapter(List<Message> messageList, Fragment fragment) {
            this.messageList = messageList;
            this.fragment = fragment;
        }

        @NonNull
        @Override
        public MyViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            View view = LayoutInflater.from(fragment.getContext()).inflate(R.layout.adapter_message_one, parent, false);
            return new MyViewHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull MyViewHolder holder, int position) {
            String message = messageList.get(position).getMessage();
            boolean isReceived = messageList.get(position).getIsReceived();
            if (isReceived) {
                holder.messageReceive.setVisibility(View.VISIBLE);
                holder.messageSend.setVisibility(View.GONE);
                holder.messageReceive.setText(message);
            } else {
                holder.messageSend.setVisibility(View.VISIBLE);
                holder.messageReceive.setVisibility(View.GONE);
                holder.messageSend.setText(message);
            }
        }

        @Override
        public int getItemCount() {
            return messageList.size();
        }

        class MyViewHolder extends RecyclerView.ViewHolder {
            TextView messageSend;
            TextView messageReceive;

            MyViewHolder(@NonNull View itemView) {
                super(itemView);
                messageSend = itemView.findViewById(R.id.message_send);
                messageReceive = itemView.findViewById(R.id.message_receive);
            }
        }
    }
}
